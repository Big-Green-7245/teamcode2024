<h1 style="text-align: center">Deerfield Robotics 2024</h1>
<p style="text-align: center">Codebase for FTC season 2024-25</p>

## Setting up the Development Environment

### Downloading FtcRobotController and Team Code

Run the following command in the terminal at the location you wish to store the project.  
This will download [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController), the library for
FTC, and [our team code](https://github.com/Big-Green-7245/teamcode2024).

```bash
git clone https://github.com/Big-Green-7245/teamcode2024
```

Then, open the project you just downloaded (called FtcRobotController) in Android Studio or Intellij.  
These two IDEs (Android Studio & Intellij) are essentially the same, and you should use Android Studio unless you know
what you're doing.

Now, you are ready to edit the Team Code located in `/TeamCode/src/main/java/org/firstinspires/ftc/teamcode`!

---

## Introduction to the Code

The main programs which the robot can execute are stored in the base `teamcode` folder.  
The `teleop` folder contains main TeleOp programs.  
The `autonomous` folder contains main autonomous programs.  
Each feature/component on the robot is implemented in its own class in the `modules` folder.  
The `state` folder offers a library to create a state machine, which is very useful in automating robots.  
The `util` folder contains miscellaneous classes that help us interact with the robot and things like telemetry.  
Roadrunner classes are located in `messages`, `tuning`, and the base `teamcode` folder.
See https://rr.brott.dev/ for how to use roadrunner.

## Resources

Many resources are available to get familiar with FTC code. Some are listed here:
* Discord (Unofficial FIRST Tech Challenge)
* [Game Manual 0](https://gm0.org/en/latest/)
* [The Cookbook](https://cookbook.dairy.foundation/)

## Uploading the code to the robot

To upload code to the robot, connect to the Control Hub via USB or WiFi-Direct.

*If connected wirelessly, you need to run this command after connecting.*

```bash
adb connect 192.168.43.1:5555
```

Once connected, it should look like this:
![Connected](.assets/connectedDevice.png)

Click the run (or rerun) button to upload the code.

---

## Trouble Shooting / Misc

### Sdk Not Found

If you get an error saying that the Android Sdk is not found, try adding `local.properties` in your
project root directory with the following line

```properties
sdk.dir=/Users/yourusernamehere/Library/Android/sdk
```

or pointing to wherever your Android Sdk is located.

### Using Java 17+ 😎

To use Java 17, two files need to be edited: `build.common.gradle` and `/FtcRobotController/build.gradle`.  
*Note that everyone running this code would need to do this.*

Locate
```groovy
compileOptions {
    sourceCompatibility JavaVersion.VERSION_1_8
    targetCompatibility JavaVersion.VERSION_1_8
}
```
in both files and replace `VERSON_1_8` with `VERSION_17`. Then reload gradle through a button that
should appear towards the top right or through the gradle menu on the right (Gradle -> Reload All Gradle Projects).

---

## Continuous Integration

This repository uses GitHub Actions to automatically build the code on every push and pull request to the main branch.  
This allows you to see if your code compiles successfully on every change.

## For Maintainers

Greetings, future captains, here are the steps you need to do to get ready for a new season.
**This requires git knowledge! Please ask for help if you are unsure.**

1. Create a new remote repository and team in the GitHub Organization
2. Add the remote repository to the team and give everyone maintainer access
3. Update the FTC Robot Controller using
   ```bash
   git remote add FtcRobotController https://github.com/FIRST-Tech-Challenge/FtcRobotController
   git fetch FtcRobotController
   git merge --squash --allow-unrelated-histories -X theirs FtcRobotController/master
   ```
4. Process the changed files and make sure not to overwrite our code (don't delete README.md for example)
5. Update the season and repository names in this `README.md`
6. Edit the local repository links in this `README.md` to the new remote repository
7. Commit and push the local repository to the new remote Repository
