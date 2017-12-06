# Robocubs' 2017 Robot Code
Welcome! This is our 2017 robot code for FIRST Steamworks.

### Gradle
We have switched over to Gradle (Pete, if you see this and don't like it, all the code before Gradle is in the branch named `gradle`.) None of the code has been changed in the time since Michigan 2017 State Championships.

#### IntelliJ IDEA and Eclipse
If you need to use Eclipse or IntelliJ IDEA, follow the steps below.

##### IntelliJ IDEA
Simply type the following in your terminal:

```bash
./gradlew idea
```

Once Gradle has downloaded itself, pulled in the project dependencies, and generated the project files, open the project using IDEA. Get coding! (IDEA support is experimental, but from testing shows almost 100% stability.)

##### Eclipse
Simply type the following in your terminal:

```bash
./gradlew eclipse
```

Once Gradle has downloaded itself, pulled in the project dependencies, and generated the project files, open the project in Eclipse. Get coding!

### Organization
The source root is the folder `src/main/java`. Within that, all the Java files are as-is from before the Gradle conversion.

### Building with Gradle <small>A Quick Overview of Targets</small>
Gradle has a few different targets that it uses.

* `./gradlew idea` generates IDEA project files. See above.
* `./gradlew eclipse` generates Eclipse project files. See above.
* `./gradlew build` creates a JAR file but does not deploy it.
* `./gradlew build deploy` creates a JAR file and then ships said JAR file to the roboRIO just as the Ant-based build process did.
* `./gradlew --offline build deploy` does the same as above but is designed so that Gradle doesn't check for updates. Useful during competition when there is limited or no Internet present.

### Changing Gradle Settings
Everything that we need is in the `build.gradle` file. Gradle handles the WPILib, OpenCV, NetworkTables, CSCore, TalonSRX and navX libraries just like before, except it organizes them for us so there is no hair pulling.

To change any deploy settings, [here](https://github.com/Open-RIO/GradleRIO) is a link to the GradleRIO plugin that handles deployment. Within the `frc` block in the `build.gradle` file is where the described settings can be changed.

### Assistance
There is no guarantee of assistance, but if you need it, file an issue.

### License
This project is licensed under the 3-clause BSD license just like WPILib.
