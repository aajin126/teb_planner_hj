def remove_execution_time_lines(input_path, output_path):
    with open(input_path, 'r') as f:
        lines = f.readlines()

    filtered = [line for line in lines if not line.startswith("Optimization time:")]

    with open(output_path, 'w') as f:
        f.writelines(filtered)

    print(f"Filtered file saved to {output_path}")


remove_execution_time_lines("execution_time_new.txt", "execution_time_new_dfiltered.txt")
