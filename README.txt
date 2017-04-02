RUNNING LIST OF RECOMMENDATIONS FOR QUADROKOPTER PROJECT CODE

Matt Woodard 18.08.2016


-RESTRUCTURING
    Whenever the opportunity presents itself, structure the new code so that it 
    is more in-line with the style of embedded software projects:

      - Create global.h where all necessary includes reside (p33EP512MU810.h,
        stdio.h, etc) and global vars/funcs, defines... Define XTAL and FCY and 
        have defines calculate CPU clock and peripheral clock
      - Hardware.h file where all hardware resources are defined... UART baud,
        SPI config, DMAs used
      - Use folders to group files belonging to same functionality. Use '.' in 
        front of folder names so they are at top of list.
      - Config bits in separate config.c file


-READABILITY
    - Keep in mind that when you finish your work on the project, the work you
    did will most likely not die off. Some aspects of your work will most likely
    be used by future students. Use good coding practices and general coding
    guidelines. Try to make your code as straightforward and readable as
    possible.
    - Use meaningful variable names and function names while keeping them as
    short as possible.
    - Everyone has their own particular way of coding, so do what feels and
    looks the best to you, but try to be consistent and try to make it obvious
    what code you added, so that the next person can easily read through your
    code once they know your coding style.


-COMMENTATION
    - Any function you create should also contain a comment block describing the
    purpose of the function. 
    - Don't comment every single line of code as it becomes quite cluttered, 
    but try to put a comment for any kind of if
    structure or calculation which requires more than a few lines of code.
    - Place comment blocks at the top of files giving a basic description of
    what the purpose of the file is and what kind of functionality it should
    contain, and at minimum the author and the date and perhaps compiler version.
    - Any old code which you are reading through or changing, try to add a few
    comments along the way so that it is easier for the next person to
    understand it.


-DEBUGGING
    - The combination of the debug environment and quadrokpter hardware makes it
    impractical to use the debug functionality of MPLAB. When creating new
    functionalities, try to include some basic debug code which allows for 
    easily running the code with and without that functionality. Easiest, but
    perhaps not best way is to use DEFINES at the top of the file which
    contains the functionality. Change the DEFINE to 1 or 0 to enable or
    disable the functionality. See main.c as an example. These debug switches
    could also be all combined into a single file 'defines.h'.


-JUDGMENT
    - Just because the code runs does NOT mean that it is perfect code. Keep in
    mind that this project was created and modified by students who are also
    learning the code as they go along.
    - Try to maintain a slight level of skepticism as you work through or re-use
    any old code. There are definitely mistakes which could stand to be fixed,
    as well as better ways to complete the same task.
    - Try to remove any pieces of code which you find to be entirely redundant
    or obsolete.


-IMPROVEMENTS
    - When re-using any code, instead of directly copying and pasting, try to
    think of a small way to improve the code. Whether by cleaning it up and 
    simplifying it, or adding additional functionality to it.
    - When adding new functionalities try to keep in mind that your code may be 
    used in another project. Try to write the code in such a way that it can 
    easily be adapted for other purposes without having to entirely rewrite it 
    just to do the same thing on another robot.


-RECOMMENDATIONS
    - Any recommendations which you have that you think would have made your life
    easier in understanding this code, add to this document or to  
    other running documentation.