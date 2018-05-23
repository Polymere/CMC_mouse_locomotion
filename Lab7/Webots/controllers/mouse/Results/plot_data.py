""" Load results data """

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from load_data import load_data


# Plotting ground contact gait

def y_options(**kwargs):
    """ Return y options """
    y_size = kwargs.pop("y_size", 4)
    y_sep = 1.0 / (y_size + 1)

    def y_pos(y): return (y + (y + 1) * y_sep) / (y_size + 1)

    return y_size, y_sep, y_pos


def add_patch(ax, x, y, **kwargs):
    """ Add patch """
    y_size, y_sep, y_pos = y_options(**kwargs)
    width = kwargs.pop("width", 1)
    height = kwargs.pop("height", y_sep)
    ax.add_patch(
        patches.Rectangle(
            (x, y_pos(y)),
            width,
            height,
            hatch='\\' if (y % 2) else '/'
        )
    )
    return


def plot_gait(time, gait, dt, **kwargs):
    """ Plot gait """
    figurename = kwargs.pop("figurename", "gait")
    fig1 = plt.figure(figurename)
    plt.title('Ground contact gait')
    ax1 = fig1.add_subplot("111", aspect='equal')
    for t, g in enumerate(gait):
        for l, gait_l in enumerate(g):
            if gait_l:
                add_patch(ax1, time[t], l, width=dt, y_size=len(gait[0, :]))
    y_values = kwargs.pop(
        "y_values",
        [
            "Left\nFoot",
            "Right\nFoot",
            "Left\nHand",
            "Right\nHand"
        ][:len(gait[0, :])]
    )
    _, y_sep, y_pos = y_options(y_size=len(gait[0, :]))
    y_axis = [y_pos(y) + 0.5 * y_sep for y in range(4)]
    plt.yticks(y_axis, y_values)
    plt.xlabel("Time [s]")
    plt.ylabel("Gait")
    plt.axis('auto')
    plt.grid(True)
    return


# Plotting joint angles
    
def plot_angles(time, joint_lh_positions, joint_rh_positions):
    plt.figure('angles')
    titles= (('Hip angles'), ('Knee angles'), ('Ankle angles'))
    legends = (('left hind limb'), ('right hind limb'))
    K = 3
    for k in np.arange(K):
        plt.subplot(K,1,k+1)
        plt.title(titles[k])
        plt.plot(time, np.rad2deg(joint_lh_positions[:, k]))
        plt.plot(time, np.rad2deg(joint_rh_positions[:, k]))
        plt.legend(legends, loc='lower right')
        plt.xlim([0, time[-1]])
        plt.ylabel('Angle [deg]')
        plt.grid('on')
    plt.xlabel('Time [s]')
    return


# Plotting muscle activations
    
def plot_activations(time, muscle_activations):
    plt.figure('activations')
    K = 8
    for k in np.arange(K):
        plt.subplot(K,1,k+1)
        plt.title('Activation muscle {}'.format(k+1))
        plt.plot(time, muscle_activations[:, k])
        plt.axis([0, time[-1], 0, 1])
        plt.ylabel('Activation')
        plt.grid('on')
    plt.xlabel('Time [s]')
    return


def main():
    """ Main """
    [time,
     ankle_l_trajectory,
     ankle_r_trajectory,
     foot_l_contact,
     foot_r_contact,
     muscle_lh_activations,
     muscle_rh_activations,
     muscle_lh_forces,
     muscle_rh_forces,
     joint_lh_positions,
     joint_rh_positions] = load_data()

    # Plot the joint angles of hip, knee and ankle joints from left and right hind limbs
    plot_angles(time, joint_lh_positions, joint_rh_positions)

    # Plot the muscle activations of either left or right hind limb
    plot_activations(time, muscle_lh_activations)

    # Plot the ground contact of both left and right hing feet
    contact_data = np.hstack((foot_r_contact, foot_l_contact))
    plot_gait(time, contact_data,  0.01)

    # Compute the duty factor of the model
    duty_factor = 0.0

    return


if __name__ == '__main__':
    plt.close('all')
    main()
    plt.show()
