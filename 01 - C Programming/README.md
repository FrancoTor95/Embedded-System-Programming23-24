# Introduction to the C programming language
The Folder **src** contains all the source code related to the examples shown during the exercises. In this folder slides can be found too.
In order to compile a .c file it is possible to use the gcc compiler.
```
gcc -o <output_file_name> <input_file_name>.c
```
For example if we want to compile the main_double_example.c we could run the following command
```
gcc -o main_double_example.exe main_double_example.c
```

# Troubleshooting: GCC not found on Windows
If you are using Windows and encounter an error like `gcc is not recognized as an internal or external command`, it means the compiler is not installed. A straightforward solution is to install MinGW, a popular compiler suite for Windows (a quick Google search will help you find it easily).
