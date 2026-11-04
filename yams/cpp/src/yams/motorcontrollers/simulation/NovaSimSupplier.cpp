// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/NovaSimSupplier.hpp"

#include <thrifty/core/MotorControl.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>
#include <wpi/math/util/MathUtil.hpp>

namespace yams::motorcontrollers::simulation {

namespace {

/** Sign of @p value: -1, 0 or 1. */
double Sign(double value) { return (value > 0.0) - (value < 0.0); }

}  // namespace

NovaSimSupplier::NovaSimSupplier(std::shared_ptr<SimSupplier> mechanism, thrifty::Nova& nova,
                                 wpi::units::second_t period)
    : m_mechanism(std::move(mechanism)), m_nova(nova), m_periodSeconds(period.value()) {}

void NovaSimSupplier::SetControlRequest(const thrifty::MotorControlRequest& request) {
  if (!m_request || m_request->GetControlType() != request.GetControlType()) {
    m_integral = 0.0;
    m_lastError.reset();
    m_profileState.reset();
  }
  m_request = request;
}

const std::optional<thrifty::MotorControlRequest>& NovaSimSupplier::GetControlRequest() const {
  return m_request;
}

double NovaSimSupplier::GetDutyCycle() const { return m_dutyCycle; }

void NovaSimSupplier::ApplyVoltage(double volts) {
  // Past a soft limit, the Nova does not drive further that way.
  auto status = m_nova.Status();
  const double position = m_mechanism->GetMechanismPosition().value();
  if (status.GetSoftLimitEnable() &&
      ((position >= status.GetSoftLimitForward() && volts > 0.0) ||
       (position <= status.GetSoftLimitReverse() && volts < 0.0)))
    volts = 0.0;
  const double supplyVolts = m_mechanism->GetMechanismSupplyVoltage().value();
  m_dutyCycle = std::clamp(volts / supplyVolts, -1.0, 1.0);
  m_mechanism->SetMechanismStatorVoltage(wpi::units::volt_t{supplyVolts * m_dutyCycle});
}

void NovaSimSupplier::UpdateSim() {
  if (m_request) {
    const bool slot1 = m_request->GetSlot() == static_cast<int>(thrifty::MotorControl::Slot::SLOT1);
    auto status = m_nova.Status();
    const double position = m_mechanism->GetMechanismPosition().value();
    const double velocity = m_mechanism->GetMechanismVelocity().value();
    // The request's feedforward, and the PID slot's F times the target.
    double feedforward = m_request->GetFeedForward() +
                         (slot1 ? status.Pid1.GetF() : status.Pid0.GetF()) * m_request->GetTarget();
    // With continuous wrapping, a position target is the equivalent nearest the mechanism, one
    // mechanism rotation apart. The Nova wraps only an absolute encoder closing its loop.
    const auto feedbackSensor = status.GetFeedbackSensor();
    const bool absoluteFeedback = feedbackSensor == thrifty::Motor::FeedbackSensorType::ABS ||
                                  feedbackSensor == thrifty::Motor::FeedbackSensorType::CAN_ENCODER;
    const double positionTarget =
        status.GetAbsWrapping() && absoluteFeedback
            ? position + wpi::math::InputModulus(m_request->GetTarget() - position, -0.5, 0.5)
            : m_request->GetTarget();
    double error = 0.0;
    // ThriftyLib gives the control type as its protocol number, so compare it with requests of each
    // type.
    if (m_request->GetControlType() == thrifty::MotorControl::VoltageVelocity(0).GetControlType()) {
      error = m_request->GetTarget() - velocity;
    } else if (m_request->GetControlType() ==
               thrifty::MotorControl::MotionProfile(0).GetControlType()) {
      const double kS = slot1 ? status.Mp1.GetKS() : status.Mp0.GetKS();
      const double kV = slot1 ? status.Mp1.GetKV() : status.Mp0.GetKV();
      const double kA = slot1 ? status.Mp1.GetKA() : status.Mp0.GetKA();
      const double kG = slot1 ? status.Mp1.GetKG() : status.Mp0.GetKG();
      const auto gravityType = slot1 ? status.Mp1.GetGravType() : status.Mp0.GetGravType();
      using Profile = wpi::math::TrapezoidProfile<wpi::units::turns>;
      const double maxVelocity = slot1 ? status.Mp1.GetMaxVel() : status.Mp0.GetMaxVel();
      const double maxAcceleration = slot1 ? status.Mp1.GetMaxAccel() : status.Mp0.GetMaxAccel();
      const Profile::State state = m_profileState.value_or(
          Profile::State{wpi::units::turn_t{position}, wpi::units::turns_per_second_t{velocity}});
      // A profile without a positive velocity and acceleration, such as before the Nova reports the
      // motion profile it was sent, holds where it is.
      const Profile::State next =
          maxVelocity > 0.0 && maxAcceleration > 0.0
              ? Profile{Profile::Constraints{wpi::units::turns_per_second_t{maxVelocity},
                                             wpi::units::turns_per_second_squared_t{maxAcceleration}}}
                    .Calculate(wpi::units::second_t{m_periodSeconds}, state,
                               Profile::State{wpi::units::turn_t{positionTarget},
                                              wpi::units::turns_per_second_t{0.0}})
              : Profile::State{state.position, wpi::units::turns_per_second_t{0.0}};
      m_profileState = next;
      // kG is the gravity with the mechanism at 0, horizontal for an arm.
      feedforward += kS * Sign(next.velocity.value()) + kV * next.velocity.value() +
                     kA * (next.velocity - state.velocity).value() / m_periodSeconds +
                     kG * (gravityType == thrifty::Motor::MPGravityType::COSINE
                               ? std::cos(2.0 * std::numbers::pi * position)
                               : 1.0);
      error = next.position.value() - position;
    } else {
      error = positionTarget - position;
    }
    if (std::abs(error) <=
        (slot1 ? status.Pid1.GetAllowableError() : status.Pid0.GetAllowableError())) {
      error = 0.0;
    }
    // The integral and the rate of change of the error, per second.
    m_integral += error * m_periodSeconds;
    const double derivative = (error - m_lastError.value_or(error)) / m_periodSeconds;
    m_lastError = error;
    ApplyVoltage((slot1 ? status.Pid1.GetP() : status.Pid0.GetP()) * error +
                 (slot1 ? status.Pid1.GetI() : status.Pid0.GetI()) * m_integral +
                 (slot1 ? status.Pid1.GetD() : status.Pid0.GetD()) * derivative + feedforward);
  }
  m_mechanism->UpdateSim();
}

bool NovaSimSupplier::GetUpdatedSim() { return m_mechanism->GetUpdatedSim(); }
void NovaSimSupplier::FeedUpdateSim() { m_mechanism->FeedUpdateSim(); }
void NovaSimSupplier::StarveUpdateSim() { m_mechanism->StarveUpdateSim(); }
bool NovaSimSupplier::IsInputFed() { return m_mechanism->IsInputFed(); }
void NovaSimSupplier::FeedInput() { m_mechanism->FeedInput(); }
void NovaSimSupplier::StarveInput() { m_mechanism->StarveInput(); }

void NovaSimSupplier::SetMechanismStatorDutyCycle(double dutyCycle) {
  m_request.reset();
  ApplyVoltage(dutyCycle * m_mechanism->GetMechanismSupplyVoltage().value());
}

wpi::units::volt_t NovaSimSupplier::GetMechanismSupplyVoltage() {
  return m_mechanism->GetMechanismSupplyVoltage();
}

wpi::units::volt_t NovaSimSupplier::GetMechanismStatorVoltage() {
  return m_mechanism->GetMechanismStatorVoltage();
}

void NovaSimSupplier::SetMechanismStatorVoltage(wpi::units::volt_t volts) {
  m_request.reset();
  ApplyVoltage(volts.value());
}

wpi::units::turn_t NovaSimSupplier::GetMechanismPosition() {
  return m_mechanism->GetMechanismPosition();
}

void NovaSimSupplier::SetMechanismPosition(wpi::units::turn_t angle) {
  m_profileState.reset();
  m_mechanism->SetMechanismPosition(angle);
}

wpi::units::turn_t NovaSimSupplier::GetRotorPosition() { return m_mechanism->GetRotorPosition(); }

wpi::units::turns_per_second_t NovaSimSupplier::GetMechanismVelocity() {
  return m_mechanism->GetMechanismVelocity();
}

void NovaSimSupplier::SetMechanismVelocity(wpi::units::turns_per_second_t velocity) {
  m_profileState.reset();
  m_mechanism->SetMechanismVelocity(velocity);
}

wpi::units::turns_per_second_t NovaSimSupplier::GetRotorVelocity() {
  return m_mechanism->GetRotorVelocity();
}

wpi::units::turns_per_second_squared_t NovaSimSupplier::GetMechanismAcceleration() {
  return m_mechanism->GetMechanismAcceleration();
}

wpi::units::turns_per_second_squared_t NovaSimSupplier::GetRotorAcceleration() {
  return m_mechanism->GetRotorAcceleration();
}

void NovaSimSupplier::SetRotorPosition(wpi::units::turn_t angle) {
  m_profileState.reset();
  m_mechanism->SetRotorPosition(angle);
}

void NovaSimSupplier::SetRotorVelocity(wpi::units::turns_per_second_t velocity) {
  m_profileState.reset();
  m_mechanism->SetRotorVelocity(velocity);
}

wpi::units::ampere_t NovaSimSupplier::GetStatorCurrent() { return m_mechanism->GetStatorCurrent(); }

wpi::units::ampere_t NovaSimSupplier::GetSupplyCurrent() { return m_mechanism->GetSupplyCurrent(); }

}  // namespace yams::motorcontrollers::simulation
