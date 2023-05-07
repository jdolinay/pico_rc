# pico_rc - Library for Raspberry Pi Pico for working with hobby servos and RC receivers.
Created by Jan Dolinay, May 2023<br/>

## Revision History
**May 2023**
+ Initial version


## Using the library
I assume you are already familiar with programming the Pico using Pico SDK. If not, please check out the excellent  documentation provided by Raspberry Pi (https://rptl.io/pico-get-started). 

Download or clone the library into some folder on your computer.

In the CMakeLists.txt file for your project add path to the library:
add_subdirectory("../../../pico_rc" pico_rc)

In the above example I use relative path to the pico_rc folder, which in my case is located three levels up from the project folder (from the CMakeLists.txt location). You will need to adjust this as needed.

Then add the library pico_rcto the target_link_libraries section of CMakeLists.txt as you would add any other SDK library. 
Here is an example:

target_link_libraries(receiver 
  pico_stdlib  
  pico_rc
)


Now you are ready to use the library in your code. Just include rc.h in your source:

\#include "pico/rc.h"

See the examples folder for example programs.


## License
This is free software; you can redistribute it and/or modify it under the terms of the MIT License.




