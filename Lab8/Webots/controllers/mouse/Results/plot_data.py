""" Load results data """

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from load_data import load_data


# Plot ground contact gait

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
    gait = np.fliplr(gait) # left foot on top of right foot
    figurename = kwargs.pop("figurename", "gait")
    fig1 = plt.figure(figurename)
    ax1 = fig1.add_subplot("111", aspect='equal')
    plt.title('Ground contact gait')
    for t, g in enumerate(gait):
        for l, gait_l in enumerate(g):
            if gait_l:
                add_patch(ax1, time[t], l, width=dt, y_size=len(gait[0, :]))
    y_values = kwargs.pop(
        "y_values",
        [
            "Right\nFoot",
            "Left\nFoot",
            #"Right\nHand",
            #"Left\nHand"
        ][:len(gait[0, :])]
    )
    _, y_sep, y_pos = y_options(y_size=len(gait[0, :]))
    y_axis = [y_pos(y) + 0.5 * y_sep for y in range(4)]
    plt.xticks(np.linspace(time[0],time[-1],5))
    plt.yticks(y_axis, y_values)
    plt.xlabel("Time [s]")
    plt.ylabel("Gait")
    plt.axis('auto')
    plt.grid(True)
    return


# Plot joint angles

def plot_angles(time, joint_lh_positions, joint_rh_positions):
    """ Plot angles """
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


# Plot joint phases

def plot_phases(joint_lh_positions, joint_rh_positions):
    """ Plot phases """
    plt.figure('phases')
    titles= (('Hip phase'), ('Knee phase'), ('Ankle phase'))
    K = 3
    for k in np.arange(K):
        plt.subplot(K,1,k+1)
        plt.title(titles[k])
        plt.plot(np.rad2deg(joint_lh_positions[:, k]), np.rad2deg(joint_rh_positions[:, k]))
        plt.xlabel('Left angle [deg]')
        plt.ylabel('Right angle [deg]')
        plt.grid('on')
    return


# Plot muscle activations
    
def plot_activations(time, muscle_activations):
    """ Plot activations """
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


# Compute duty factor

def duty_cycle(foot_contact, nb_precycles, filter_threshold):
    """ Compute duty cycle """
    duty_cycle = []
    step = 0
    nb_ground = 0
    nb_air = 0
    for t, g in enumerate(foot_contact):
        if g and t > 0 and not foot_contact[t-1]:
            if nb_ground >= filter_threshold and nb_air >= filter_threshold:
                if len(foot_contact) > t+filter_threshold and np.sum(foot_contact[t:t+filter_threshold]) == filter_threshold:
                    step += 1
                    if step > nb_precycles:
                        duty_cycle.append(float(nb_ground)/float(nb_ground+nb_air))
                    nb_ground = 0
                    nb_air = 0
                else:
                    foot_contact[t:t+filter_threshold] = 0
                    g = foot_contact[t]
            else:
                k = 0
                if nb_ground < nb_air:
                    while nb_ground > 0:
                        if foot_contact[t-k]:
                            foot_contact[t-k] = 0
                            nb_air += 1
                            nb_ground -= 1
                        k += 1
                else:
                    while nb_air > 0:
                        if not foot_contact[t-k]:
                            foot_contact[t-k] = 1
                            nb_ground += 1
                            nb_air -= 1
                        k += 1
                g = foot_contact[t]
        if g:
            nb_ground += 1
        else:
            nb_air += 1
    return duty_cycle

def compute_dutyfactor(foot_l_contact, foot_r_contact, nb_precycles, filter_threshold):
    """ Compute duty factor """
    duty_factor = [0.0, 0.0]
    duty_cycles_l = 0.0
    duty_cycles_r = 0.0
    duty_cycles_l = duty_cycle(foot_l_contact, nb_precycles[0], filter_threshold)
    duty_cycles_r = duty_cycle(foot_r_contact, nb_precycles[1], filter_threshold)
    duty_factor[0] = np.mean(duty_cycles_l)
    duty_factor[1] = np.mean(duty_cycles_r)
    return duty_factor


# Process data

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
    
    # Plot the joint phases of hip, knee and ankle joints from left and right hind limbs
    plot_phases(joint_lh_positions, joint_rh_positions)

    # Plot the muscle activations of either left or right hind limb
    plot_activations(time, muscle_lh_activations)

    # Plot the ground contact of both left and right hing feet
    dt = 0.01
    contact_data = np.hstack((foot_l_contact, foot_r_contact))
    plot_gait(time, contact_data,  dt)

    # Compute the duty factor of both left and right hing feet
    nb_precycles = [1, 1] # number of unstable cycles before stable gait [left, right]
    filter_threshold = 20 # threshold for filtering the noisy peaks in measurements
    duty_factor = compute_dutyfactor(foot_l_contact, foot_r_contact, nb_precycles, filter_threshold)
    print(duty_factor)

    return


if __name__ == '__main__':
    plt.close('all')
    main()
    plt.show()
