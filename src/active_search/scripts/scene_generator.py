import os
import glob

from active_search.search_sim import Simulation

def main():
    gui = True
    scene_id = "random" 
    sim = Simulation(gui, scene_id)

    while True:

        sim.reset()

        user_input = input("Press S to save: ")

        print("You input", user_input)

        if user_input.lower() == "s" or user_input.upper() == "S":
            test_name = get_latest_test()
            if test_name is not None:
                sim.scene.save_scene_to_yaml(test_name)




def get_latest_test() -> str:
    directory_path = "/home/pitcher/dev_ws/thesis_ws/active_search/src/active_search/cfg/sim"
    pattern = "test_*.yaml"

    # Get a list of matching files in the directory
    matching_files = glob.glob(os.path.join(directory_path, pattern))

    print("Matching files:", matching_files)

    # Sort the files by modification time (latest first)
    sorted_files = sorted(matching_files, key=lambda x: int(os.path.splitext(os.path.basename(x))[0].split('_')[-1]))

    if sorted_files:
        latest_file = sorted_files[-1]
        # Extract the number from the latest file
        file_number = int(latest_file.split('_')[-1].split('.')[0])
        # Increment the number for the new file name
        new_file_name = "test_" + str(file_number + 1) + ".yaml"
        print("Latest test file:", latest_file)
    else:
        print("No matching files found.")
        new_file_name = None

    return new_file_name


if __name__ == "__main__":
    main()