---
description: Follow the YAMS and YAGSL tutorials beside your code in WPILib VS Code.
icon: book-open
---

# Install the YASS Tutorials extension

The YASS Tutorials extension brings these tutorials into WPILib VS Code. It adds a **YASS** tab that lists every YAMS and YAGSL tutorial step by step, and opens each step beside your code. The tutorials are the same as on this site, so the extension is optional.

{% stepper %}
{% step %}
### Download the extension

Download [yass-tutorials.vsix](https://github.com/Yet-Another-Software-Suite/yass-tutorials-vscode/releases/download/latest/yass-tutorials.vsix).
{% endstep %}

{% step %}
### Install it in WPILib VS Code

1. Click **Extensions** in the activity bar on the left.
2. Click **...** at the top of the Extensions view.
3. Click **Install from VSIX...**, pick the `yass-tutorials.vsix` you downloaded and click **Open**.

**YASS Tutorials** appears in the list of installed extensions, and a book icon appears in the activity bar.

<figure><img src="../../.gitbook/assets/yass-tutorials-install.gif" alt="In WPILib VS Code, the user clicks Extensions in the activity bar, then the ... button at the top of the Extensions view, then Install from VSIX. In the file picker they pick yass-tutorials.vsix in Downloads and click Open. YASS Tutorials appears in the installed extensions, a book icon appears in the activity bar, and a message says Completed installing extension."><figcaption><p>Installing yass-tutorials.vsix from the Extensions view.</p></figcaption></figure>

{% hint style="info" %}
Install it in WPILib VS Code, the one the WPILib installer adds. The extension uses WPILib's extension to create robot projects and install vendordeps.
{% endhint %}
{% endstep %}

{% step %}
### Open a tutorial

Click the book icon. Under **Tutorials**, click the arrow beside a tutorial to see its steps, then click a step to open it beside your code.

<figure><img src="../../.gitbook/assets/yass-tutorials-open-step.gif" alt="With RobotContainer.java open, the user clicks the book icon. The YASS tab shows Your Robot Project, with MyRobot, Java, Commands v2, YAMS and YAGSL all checked, and the YAMS and YAGSL tutorials below. The user clicks the arrow beside Arm, which lists its six steps, and clicks Step 1: The motor controller. The step opens to the right of RobotContainer.java."><figcaption><p>Opening the Arm tutorial's first step beside the code.</p></figcaption></figure>
{% endstep %}
{% endstepper %}

## Use the extension

### Install YAMS and YAGSL

**Your Robot Project** shows the open project and whether it has YAMS and YAGSL. Click **YAMS** or **YAGSL** to install the vendordep that matches your project: Commands v2 or Commands v3, Java or C++. Then click **Build Robot Code** so that the library downloads.

<figure><img src="../../.gitbook/assets/yass-tutorials-install-yams.gif" alt="The user clicks the book icon. Your Robot Project shows MyRobot, Java, Commands v2, with YAMS and YAGSL marked click to install and no vendor libraries yet. The user clicks YAMS. It changes to YAMS 2026.11.14 with a check mark, and a message says Installed YAMS for Java, Commands v2, Build the robot code once so it downloads, with a Build Robot Code button."><figcaption><p>Installing YAMS from Your Robot Project.</p></figcaption></figure>

When a tutorial's hardware needs a vendor library your project doesn't have, the step says so at the top. Click **Open vendor libraries**, find the library in WPILib's list and click **Install**.

<figure><img src="../../.gitbook/assets/yass-tutorials-needs-library.png" alt="A box at the top of a tutorial step reads: MyRobot needs REVLib. This tutorial&#x27;s hardware uses REVLib. Install it before building the code. Below it are an Open vendor libraries button and the line: Find REVLib in WPILib&#x27;s vendor library list, and click Install."><figcaption><p>A step that needs REVLib.</p></figcaption></figure>

### Create files and insert code

When a step says to create a file, click its **Create** button. The extension writes the file to the right folder, starts it so that it compiles, and opens it beside the tutorial.

Each code sample has **Copy** and **Insert**. Click where the code goes in your file, then click **Insert** to put the sample at the cursor.

<figure><img src="../../.gitbook/assets/yass-tutorials-create-insert.gif" alt="On Arm, Step 1, the user clicks Create ArmSubsystem.java. ArmSubsystem.java opens to the right of the tutorial with its package, import and an empty ArmSubsystem class. The user clicks inside the class, scrolls the tutorial to the Start the config sample and clicks Insert. The sample&#x27;s SmartMotorControllerConfig lines appear inside the class."><figcaption><p>Creating ArmSubsystem.java, then inserting a sample into it.</p></figcaption></figure>

### Pick your code language

The **Code** list at the top of every step switches every sample to **Java, Commands v2**, **Java, Commands v3** or **C++**. It starts at **From my project**, which matches the project you have open.

<figure><img src="../../.gitbook/assets/yass-tutorials-code-language.gif" alt="The user opens the Code list at the top of a step, which offers From my project (Java, Commands v2), Java, Commands v2, Java, Commands v3 and C++, and picks C++. The samples change to C++, such as cpp/subsystems/ArmSubsystem.cpp with WithFeedback and WithFeedforward."><figcaption><p>Switching the samples to C++.</p></figcaption></figure>

### Track your progress and search

Hover over a step in **Tutorials** and click its check mark when you finish it. Each tutorial counts the steps you have done. To find a topic, click the magnifying glass at the top of **Tutorials** and type what you are looking for, then click a result to open it.

<figure><img src="../../.gitbook/assets/yass-tutorials-done-search.gif" alt="The user hovers over Arm, Step 1: The motor controller and clicks its check mark. The step gets a green check mark and Arm changes from 0/6 to 1/6. The user clicks the magnifying glass at the top of Tutorials and types flywheel speed. The result Step 2: Commands and buttons, with the line One command sets both flywheel speeds, appears. The user clicks it and the Double FlyWheel step opens."><figcaption><p>Marking a step as done, then searching for flywheel speed.</p></figcaption></figure>

## Updates

The tutorials update themselves: the extension checks for new ones every day and keeps the ones it has for when you are offline. Click **Refresh** at the top of **Tutorials** to check right away.

The extension itself doesn't update on its own. To get new features, download `yass-tutorials.vsix` again and install it the same way.
