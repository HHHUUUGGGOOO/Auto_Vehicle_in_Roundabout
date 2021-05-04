# Auto_Vehicle_in_Roundabout
Solve the automatic vehicles' scheduling in roundabout

# Compile (MobaXterm)
# How to build? 
- cd Auto_Vehicle_in_Roundabout
- cd src
- make clean
- make

# How to run?
- cd Auto_Vehicle_in_Roundabout
- ./bin/runner -case<index> input/v_in/<input_file_name> input/ra_input/<roundabout_infomation_file_name> output/<output_file_name>

# How to eliminate binary file?
- cd Auto_Vehicle_in_Roundabout
- cd src
- make clean

# How to generate vehicle input file?
- ./generator/vechile_input_generator.py

# How to generate roundabout information file?
- ./generator/ra_input_generator.py

# How to plot?
- ./plotter/plot_answer.py

# How to check output file?
- ./checker/output_verifier.py



