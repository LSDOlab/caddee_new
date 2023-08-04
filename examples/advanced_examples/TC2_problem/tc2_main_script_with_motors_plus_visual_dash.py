if __name__ == '__main__':
    from tc2_main_script_with_motors_plus_visual import TC2DB
     # Build a standard dashbaord object

    dash_object = TC2DB().assemble_basedash()
    # uncomment to produces images for all frames
    # dash_object.visualize()
    # uncomment to produces images for n_th frame
    # n = [0, 2, 4, 6, 8, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100]
    # dash_object.visualize(frame_ind = n, show = False)
    # uncomment to produces image for last frame
    dash_object.visualize_most_recent(show = True)
    # uncomment to make movie
    # dash_object.visualize_all()
    # dash_object.make_mov()
    # uncomment to run gui
    # dash_object.run_GUI()