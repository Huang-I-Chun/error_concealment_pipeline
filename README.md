# PC_error_concealment

## Environment requirement
Cpp point cloud library (https://pointclouds.org/downloads/),

Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page#Download),

Cmake (https://cmake.org/install/)

## How to build the project using cmake
Compile execution file `Pipeline` under a new folder `build`
```bach
git clone git@github.com:Hunag-I-Chun/error_concealment_pipeline.git
cd error_concealment_pipeline
mkdir build
cd build
cmake ..
make
```

## Usage
Go back to folder `error_concealment_pipeline`
```bash
cd ..
```

Run following command:
```bash
python client.py -prev PREVIOUS_FRAME_PATH -next NEXT_FRAME_PATH -out OUTPUT_PATH -pos RELATIVE_POSITION
``` 

Example command:
```bash
python client.py -prev longdress_1051.ply -next longdress_1055.ply -out build/output.ply -pos 0.5
```
`-prev longdress_1051.ply` previous point cloud file we provide

`-next longdress_1055.ply` next point cloud file we provide

`-out build/output.ply` output point cloud file path

`-pos 0.5` means the output file is right at the middle of previous and next frames

## Different pipeline sample
In `error_concealment_pipeline/config.hpp`, we've provided P, B, Q pipeline combinations for your convenience. The current pipeline in use is labeled as "Q." To switch to a different pipeline combination, simply comment or uncomment the relevant lines based on your requirements. Remember that every time you make changes to either the .hpp or .cpp files, you'll need to rebuild the project.

## How to create your own combination/stage method
To create custom methods, please consult the `error_concealment_pipeline/base.hpp` file. Within `base.hpp`, we've established a virtual class called Stage, which all methods must inherit from. Additionally, in the `Pipeline_Object` definition, we've clearly specified the output variables for each stage. It's essential to ensure that any custom stage methods you create align with the input and output requirements of each respective stage. After implement your own stage method, modify the pipeline reference in `error_concealment_pipeline/config.hpp`, and rebuild the project.

## Other tips
Given that various point cloud datasets may employ different coordinate systems and units of measurement, we recommend scaling your dataset to align with the MPEG (http://plenodb.jpeg.org/pc/8ilabs/) for consistency.
