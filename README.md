# Auto_Vehicle_in_Roundabout
Solve the automatic vehicles' scheduling in roundabout

# Compile (MobaXterm)
# How to build? 
- cd Auto_Vehicle_in_Roundabout
- make clean
- make

# How to run?
- cd Auto_Vehicle_in_Roundabout
- ./bin/runner -case<index> input/v_in/<input_file_name> input/ra_in/<roundabout_infomation_file_name> output/<output_file_name>

# How to eliminate binary file?
- cd Auto_Vehicle_in_Roundabout
- make clean

# How to generate vehicle input file?
- Use 'python3 ./generator/vechile_input_generator.py -h' to get input arguments

# How to generate roundabout information file?
- Use 'python3 ./generator/ra_input_generator.py -h' to get input arguments

# How to plot?
- python3 ./plotter/plot_answer.py --output_file output/<output_file_name> --ra_file input/ra_in/<roundabout_infomation_file_name>
- python3 ./plotter/plot_scheduling_time.py --input_file input/v_in/<vehicle_input_file_name>

# How to check output file?
- cd checker
- make clean
- make
- ./checker ../input/v_in/<vehicle_input_file_name> ../input/ra_in/<roundabout_infomation_file_name> ../output/<output_file_name>



