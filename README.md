# bvhMotionIK
 This program uses IK to compute each joint's angle in a given BVH file and generate a new BVH file with all frames stored in it.
## How to Compile
 1. Use the command "git clone [GitHub URL]" to clone and download the project from GitHub to your computer.
 2. Run "setup.bat" to generate an "IK_processer.sln" file.
 3. You can open the "IK_processer.sln" file using visual studio. Now you can compile and run the program.
## How to Run
### Visual Studio
 You can directly compile and run the code in Visual Studio. Don't forget to add arguments in Visual Studio.
 
 To set command-line arguments in Visual Studio, right-click on the project name, then go to Properties. In the Properties Pane, go to "Debugging", and in this pane is a line for "Command-line arguments." Add the values you would like to use on this line.
 
 This program requires three arguments. 
 * First is the path of the motion_position file which contains all positions of each joint. This should be a .txt file.
 * Second is the path of the base BVH file which contains the skeleton and base motion. This should be a .bvh file.
 * The third is the output file name that we want. This should be a .bvh file.
 ### Executable File
 There is a .exe file in the "APP_release" folder. You can directly double-click "run.bat" to run the executable file.
 
 This .bat file uses the file in "position", "bvh" to generate the output .bvh file and store it in the "output" folder.
 
 You can change the path in .bat file to generate your own .bvh file.
 
 ## Important
 The position file is in .npy format at the beginning. The "npy2txt.py" in the "APP_release" folder is used to convert it into a .txt format. You can see how it works by checking the "run.bat" file.
