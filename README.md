# Springs

## Description:
This project is meant to be a CPP version of this Python_Project[https://github.com/PurplePegasuss/springs_project.git]


## Requirements
1. Open3d
2. NumCpp
3. Python-3.10
4. C++14 or above
5. Yaml-cpp


## Download the following readily build libraries:
1. open3d[www.google.com] (change this link after uploading it to cloud later)
2. NumCpp
  
    1.  
        ```shell
        choco install boost-msvc-14.3
       ```
  
   
     2. ```shell
        cd <with the same directory alongside Springs>
        git clone https://github.com/dpilger26/NumCpp.git
       
        ```
   
     3. ```shell
         cd NumCpp
         mkdir build
         cd build
         cmake ..

         ```
     4. 
        ```shell
          cmake --build . --target install
        ```
     5. ```shell
            cd ..
            cp -r .\include\* <path_to_springs_include_folder>
        ```   


3. Yaml-cpp[www.google.com] (To be installed and integrated)

    1.  
        ```shell
      https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.6.0.zip
       ```
    2. 
        ```shell
        cd yaml-cpp
        mkdir build
        cd build
        cmake -DCMAKE_INSTALL_PREFIX="your_open3d_install_folder_location>\yaml-cpp_install" ..
        cmake --build . --target install
       ```
    3.
      ```shell
        cd ../..
        cd yaml-cpp_install/include/* <path_to_springs_include_folder>
       ```


### How to Run
   ```shell
    cd Springs
    mkdir build
    cd build
    cmake -DOpen3D_ROOT="<your_open3d_install_folder_location>\open3d_install" ..
    cmake --build . --config Release --parallel 12
  ```

