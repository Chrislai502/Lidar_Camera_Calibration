import tkinter as tk
import params
import transformations

# Creates the GUI
def create_slider():
    # Nested function to write slider values to params file
    def show_overlay_trans_x(value):
        params.usr_translation = (float(value), w2.get(), w3.get())
    def show_overlay_trans_y(value):
        params.usr_translation = (w1.get(), float(value), w3.get())
    def show_overlay_trans_z(value):
        params.usr_translation = (w1.get(), w2.get(), float(value))
    def show_overlay_rot_x(value):
        params.usr_rotation = (float(value), w5.get(), w6.get())
    def show_overlay_rot_y(value):
        params.usr_rotation = (w4.get(), float(value), w6.get())
    def show_overlay_rot_z(value):
        params.usr_rotation = (w4.get(), w5.get(), float(value))
    def show_overlay_cx(value):
        params.usr_cx_scale = int(value)
    def show_overlay_cy(value):
        params.usr_cy_scale = int(value)
    def show_overlay_fx(value):
        params.fx = value
    

    def show_overlay():
        params.usr_translation = (w1.get(), w2.get(), w3.get())
        params.usr_rotation = (w4.get(), w5.get(), w6.get())
        
    # Nested function to print out the slider values and total rot / trans values  as well as the calibation matrix to the terminal
    # Saves all the calibration parameters to a .txt file called calibrations.txt
    def print_calibration_parameters():
        translation_tot = ((params.translation[0]+w1.get()), (params.translation[1]+w2.get()),(params.translation[2]+w3.get()))
        rotation_tot = ((params.rotation[0]+w4.get()), (params.rotation[1]+w5.get()), (params.rotation[2]+w6.get()))
        matrix = transformations.matrix(rotation_tot, translation_tot)
        translation_gui = (w1.get(), w2.get(), w3.get()) 
        rotation_gui = (w4.get(), w5.get(), w6.get()) 
        print ("The translation you added from the sliders is:", translation_gui)
        print ("The rotation you added from the sliders is:", rotation_gui)
        print("The total calibration matrix is:")
        print (matrix[0])
        print (matrix[1])
        print (matrix[2])
        
        # Writing inputs to .txt file
        txt = open ("calibrations.txt", "a")
        txt.write("########################### \n")
        txt.write("\n")
        txt.write("The calibration matrix is: \n")
        txt.write(str(matrix[0]))
        txt.write("\n")
        txt.write(str(matrix[1]))
        txt.write("\n")
        txt.write(str(matrix[2]))
        txt.write("\n")
        txt.write("\n")
        txt.write("The initial translation is: \n")
        txt.write(str(params.translation))
        txt.write("\n")
        txt.write("The translation you entered is: \n")
        txt.write(str(translation_gui))
        txt.write("\n")
        txt.write("\n")
        txt.write("The initial rotation is: \n")
        txt.write(str(params.rotation))
        txt.write("\n")
        txt.write("The rotation you entered is: \n")
        txt.write(str(rotation_gui))
        txt.write("\n")


    # Definition of sliders
    master = tk.Tk()
    master.title("Calibration Tool")

    w1 = tk.Scale(master, label = "Translation along x-axis (meters):", from_ = params.min_trans, to = params.max_trans, \
        length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_trans, command=show_overlay_trans_x)
    w1.set(params.usr_translation[0])
    w1.pack()
    w2 = tk.Scale(master, label = "Translation along y-axis (meters):", from_ = params.min_trans, to = params.max_trans, \
        length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_trans, command=show_overlay_trans_y)
    w2.set(params.usr_translation[1])
    w2.pack()
    w3 = tk.Scale(master, label = "Translation along z-axis (meters):", from_ = params.min_trans, to = params.max_trans, \
        length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_trans, command=show_overlay_trans_z)
    w3.set(params.usr_translation[2])
    w3.pack()

    w4 = tk.Scale(master, label = "Rotation around x-axis (degrees) (Rotation order: z, y', x''):", \
        from_ = params.min_deg, to = params.max_deg, length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_rot, command=show_overlay_rot_x)
    w4.set(params.usr_rotation[0])
    w4.pack()

    w5 = tk.Scale(master, label = "Rotation around y-axis (degrees):", from_ = params.min_deg, to = params.max_deg, \
        length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_rot, command=show_overlay_rot_y)
    w5.set(params.usr_rotation[1])
    w5.pack()
    
    w6 = tk.Scale(master, label = "Rotation around z-axis (degrees):", from_ = params.min_deg, to = params.max_deg, \
        length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_rot, command=show_overlay_rot_z)
    w6.set(params.usr_rotation[2])
    w6.pack()

    # ---------------------------------------------------------------------------- #
    #                          Chris Additional Parameters                         #
    # ---------------------------------------------------------------------------- #
    w7 = tk.Scale(master, label = "Scaling on cx:", from_ = params.min_px, to = params.max_px, \
    length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_px, command=show_overlay_cx)
    w7.set(params.usr_cx_scale)
    w7.pack()

    w8 = tk.Scale(master, label = "Scaling on cy:", from_ = params.min_px, to = params.max_px, \
    length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_px, command=show_overlay_cy)
    w8.set(params.usr_cy_scale)
    w8.pack()

    w9 = tk.Scale(master, label = "Scaling on Fx:", from_ = params.minx, to = params.maxx, \
    length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_rot, command=show_overlay_fx)
    w9.set(params.fx)
    w9.pack()

    # Definition of buttons
    tk.Button(master, text = '          Show Overlay          ', command = show_overlay).pack()
    tk.Button(master, text = 'Print Calibration Parameters', command = print_calibration_parameters).pack()
    tk.mainloop()
    

def main(args=None):
    create_slider()

if __name__ == '__main__':
    main()
