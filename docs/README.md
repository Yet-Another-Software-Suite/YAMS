---
description: >-
  Welcome to Yet Another Mechanism System! A modern and easy solution to
  mechanism programming!
layout:
  width: wide
  title:
    visible: false
  description:
    visible: false
  tableOfContents:
    visible: false
  outline:
    visible: false
  pagination:
    visible: false
  metadata:
    visible: true
---

# Yet Another Mechanism System

<h2 align="center">Yet Another Mechanism System</h2>

<p align="center">Welcome to your team’s new mechanism system</p>

<table data-card-size="large" data-view="cards"><thead><tr><th></th><th></th><th></th><th data-hidden data-card-target data-type="content-ref"></th><th data-hidden data-card-cover data-type="image">Cover image</th></tr></thead><tbody><tr><td><i class="fa-leaf">:leaf:</i></td><td><strong>Easy interface</strong></td><td>Get started with the developer platform in 5 minutes.</td><td><a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ZM0CFmYiQzcrY4zDcTtZ/">Documentation</a></td><td><a href="https://images.unsplash.com/photo-1515879218367-8466d910aaa4?crop=entropy&#x26;cs=srgb&#x26;fm=jpg&#x26;ixid=M3wxOTcwMjR8MHwxfHNlYXJjaHwxfHxjb2RlfGVufDB8fHx8MTc1NjgzODk4Nnww&#x26;ixlib=rb-4.1.0&#x26;q=85">https://images.unsplash.com/photo-1515879218367-8466d910aaa4?crop=entropy&#x26;cs=srgb&#x26;fm=jpg&#x26;ixid=M3wxOTcwMjR8MHwxfHNlYXJjaHwxfHxjb2RlfGVufDB8fHx8MTc1NjgzODk4Nnww&#x26;ixlib=rb-4.1.0&#x26;q=85</a></td></tr><tr><td><i class="fa-terminal">:terminal:</i></td><td><strong>API reference</strong></td><td>Browse, test, and implement APIs.</td><td><a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ezOwaXLQ3h1N7tr3zYnj/">API Reference</a></td><td><a href="https://images.unsplash.com/photo-1509475826633-fed577a2c71b?crop=entropy&#x26;cs=srgb&#x26;fm=jpg&#x26;ixid=M3wxOTcwMjR8MHwxfHNlYXJjaHw0fHxEb2N1bWVudGF0aW9ufGVufDB8fHx8MTc1NjgzOTQzOXww&#x26;ixlib=rb-4.1.0&#x26;q=85">https://images.unsplash.com/photo-1509475826633-fed577a2c71b?crop=entropy&#x26;cs=srgb&#x26;fm=jpg&#x26;ixid=M3wxOTcwMjR8MHwxfHNlYXJjaHw0fHxEb2N1bWVudGF0aW9ufGVufDB8fHx8MTc1NjgzOTQzOXww&#x26;ixlib=rb-4.1.0&#x26;q=85</a></td></tr></tbody></table>

{% columns %}
{% column %}
**Get started in 30 minutes**

Less guesswork, no complexity—just your first successful mechanism, fast.

Setting up your first mechanism should be easy and painless and thats exactly what YAMS aims to provide.

YAMS provides an effortless programming experience by neatly dividing the process into distinct steps: SmartMotorController Configuration, SmartMotorController creation, Mechanism configuration, and Mechanism creation. This structured approach simplifies the integration of mechanisms like Arms, Elevators, Turrets, and more, allowing developers to effortlessly manage and control them using commands from Mechanism classes. Built exclusively for WPILib command-based programming, YAMS also offers sophisticated features such as advanced telemetry and live tuning, ensuring a streamlined and efficient development process for all supported mechanisms.

<a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ZM0CFmYiQzcrY4zDcTtZ/" class="button primary" data-icon="rocket-launch">Get started</a> <a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ezOwaXLQ3h1N7tr3zYnj/" class="button secondary" data-icon="terminal">API reference</a>
{% endcolumn %}

{% column %}
{% code title="ArmSubsystem.java" %}
```java
// Create your motor controller config
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimit(Degrees.of(-30), Degrees.of(100))
      .withGearing(gearing(gearbox(3, 4)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);

// Create your motor
TalonFXS                   armMotor    = new TalonFXS(1);
SmartMotorController       motor       = new TalonFXSWrapper(armMotor, DCMotor.getNEO(1), motorConfig);

// Create your mechanism config
ArmConfig                  config    = new ArmConfig(motor)
      .withLength(Meters.of(0.135))
      .withHardLimit(Degrees.of(-100), Degrees.of(200))
      .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(1))
      .withStartingPosition(Degrees.of(0))
      .withHorizontalZero(Degrees.of(0));

// Create the Arm!
Arm                        arm = new Arm(config);
```
{% endcode %}
{% endcolumn %}
{% endcolumns %}

{% columns %}
{% column %}
<figure><img src=".gitbook/assets/YAMS.png" alt=""><figcaption></figcaption></figure>
{% endcolumn %}

{% column %}
**Learn more about Yet Another Mechanism System**

Read guides, watch tutorials, and learn more about working with the developer platform and integrating it with your own stack.

<a href="https://app.gitbook.com/s/ZM0CFmYiQzcrY4zDcTtZ/tutorials" class="button primary" data-icon="book-open">Guides</a> <a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ZM0CFmYiQzcrY4zDcTtZ/" class="button secondary" data-icon="book">Documentation</a>
{% endcolumn %}
{% endcolumns %}

<h2 align="center">Join a community of over 400 teams!</h2>

<p align="center">Join our Discord community or create your first PR in just a few steps.</p>

<table data-card-size="large" data-view="cards"><thead><tr><th></th><th></th><th></th><th></th><th data-hidden data-card-cover data-type="image">Cover image</th></tr></thead><tbody><tr><td><i class="fa-discord">:discord:</i></td><td><strong>Discord community</strong></td><td>Join our Discord community to post questions, get help, and share resources with over 3,000 like-minded developers.</td><td><a href="https://discord.gg/yagsl" class="button secondary">Join Discord</a></td><td><a href="https://images.unsplash.com/photo-1679057001914-59ab4131dfff?crop=entropy&#x26;cs=srgb&#x26;fm=jpg&#x26;ixid=M3wxOTcwMjR8MHwxfHNlYXJjaHwyfHxkaXNjb3JkfGVufDB8fHx8MTc1NjgzODk5M3ww&#x26;ixlib=rb-4.1.0&#x26;q=85">https://images.unsplash.com/photo-1679057001914-59ab4131dfff?crop=entropy&#x26;cs=srgb&#x26;fm=jpg&#x26;ixid=M3wxOTcwMjR8MHwxfHNlYXJjaHwyfHxkaXNjb3JkfGVufDB8fHx8MTc1NjgzODk5M3ww&#x26;ixlib=rb-4.1.0&#x26;q=85</a></td></tr><tr><td><i class="fa-github">:github:</i></td><td><strong>GitHub</strong></td><td>Our product is 100% open source and built by developers just like you. Head to our GitHub repository to learn how to submit your first PR.</td><td><a href="https://github.com/Yet-Another-Software-Suite/YAMS/pulls" class="button secondary">Submit a PR</a></td><td><a href=".gitbook/assets/YASS_LOGO.png">YASS_LOGO.png</a></td></tr></tbody></table>
