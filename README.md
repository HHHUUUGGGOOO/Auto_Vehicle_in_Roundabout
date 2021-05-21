# Auto_Vehicle_in_Roundabout
Solve the automatic vehicles' scheduling in roundabout

# Compile (MobaXterm)
# How to build? 
- cd Auto_Vehicle_in_Roundabout
- make clean
- make

# How to run?
- cd Auto_Vehicle_in_Roundabout
- ./bin/runner -case<index> input/v_in/<input_file_name> input/ra_input/<roundabout_infomation_file_name> output/<output_file_name>

# How to eliminate binary file?
- cd Auto_Vehicle_in_Roundabout
- make clean

# How to generate vehicle input file?
- python3 ./generator/vechile_input_generator.py

# How to generate roundabout information file?
- python3 ./generator/ra_input_generator.py

# How to plot?
- python3 ./plotter/plot_answer.py

# How to check output file?
- python3 ./checker/output_verifier.py input/v_input/<input_file_name> input/ra_input/<roundabout_infomation_file_name> output/<output_file_name>



