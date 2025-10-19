
Using Source Code
	- review source code video https://youtu.be/2VGkgCKXVc0
	1. Install Python  
	2. Open CMD window and navigate to the folder python was installed in.
	3. Load each of the following Python modules:
		python -m pip install pyserial
		python -m pip install inputs
		python -m pip install numpy
		python -m pip install ttkthemes
		python -m pip install opencv-python
		python -m pip install Pillow
		python -m pip install auto-py-to-exe
		python -m pip install matplotlib
		python -m pip install pygrabber
		python -m pip install datetime
		python -m pip install pathlib
		python -m pip install scipy
		python -m pip installttkbootstrap
		
		
    4. Compile kinematics.cpp
		open cmd promt - "x64 Native Tools Command Prompt for VS 2022"
		cd "C:\Users\Chris\Desktop\AR4-MK3\AR4-MK3 Software\AR4 HMI interface 6.0 source"
		rmdir /s /q build
		mkdir build
		cd build
		cmake .. -A x64 -Dpybind11_DIR="C:/Users/Chris/AppData/Local/Programs/Python/Python312/Lib/site-packages/pybind11/share/cmake/pybind11"
		cmake --build . --config Release


	5. To convert ARCS.py to an EXE file python open CMD window and navigate to the folder python was installed in.
		and then install program: -m pip install auto-py-to-exe.
		Next execute the execute py to exe program by typing python -m auto_py_to_exe in cmd window.
		add each of these to Advanced Options --Hidded-import
			vtkmodules.all
			vtkmodules.util.execution_model
			vtkmodules.vtkRenderingTk
			vtkmodules.vtkInteractionStyle
			vtkmodules.vtkRenderingCore
			vtkmodules.vtkCommonCore
			vtkmodules.vtkCommonDataModel
			vtkmodules.vtkCommonExecutionModel
			vtkmodules.util.data_model
		Use this program to create the exe files. Copy all .ico, .gif and program files from source code folder into your new exe folder otherwise exe will not work.
		