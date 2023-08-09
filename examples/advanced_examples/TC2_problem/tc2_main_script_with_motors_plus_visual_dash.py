import numpy as np


if __name__ == '__main__':
    from tc2_main_script_with_motors_plus_visual import TC2DB
     # Build a standard dashbaord object

    dash_object = TC2DB().assemble_basedash()
    dash_object.fps = 10
    dash_object.use_timestamp(date='2023-08-04', time='23_16_31')
    # uncomment to produces images for all frames
    # dash_object.visualize()
    # uncomment to produces images for n_th frame
    # n_1 = np.arange(1, 100, 2).tolist()
    n_1 = np.arange(0, 100, 1).tolist()
    n_2 = np.arange(105, 200, 5).tolist()
    n_3 = np.arange(210, 300, 10).tolist()
    n_4 = np.arange(300, 1200, 50).tolist()
    # n_4 = np.arange(320, 900, 20).tolist()

    n = n_1 + n_2 +  n_3 + n_4 
    # n = [0, 2, 4, 6, 8, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100]
    dash_object.visualize(frame_ind = n, show = False)
    # uncomment to produces image for last frame
    # dash_object.visualize_most_recent(show = True)
    # uncomment to make movie
    # dash_object.visualize_all()
    # dash_object.make_mov()
    # uncomment to run gui
    # dash_object.run_GUI()