#!/usr/bin/env python
import rospy
from hopper_training.msg import QLearnMatrix
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time
"""
Number of possible states = pow(Splits_Values,Number_Vars_Observed)

{
('5.05.06.05.01.06.06.010.0', 5): 91.12787228202268,
('6.04.06.04.01.05.06.09.0', 2): 97.58134612559945,
('6.04.06.04.01.06.06.09.0', 1): 90.54428575740964,
('5.04.05.05.09.04.06.04.0', 1): 91.04102460695779,
('5.05.05.05.09.05.06.09.0', 5): 91.91467257108287,
('5.05.05.05.01.05.06.07.0', 1): 78.6968473161037,
('5.04.06.05.08.03.05.01.0', 2): 89.44222104760014,
('5.05.07.04.01.05.05.010.0', 2): 90.26279590296546,
('6.04.07.04.01.05.06.010.0', 1): 89.5333672515356,
('6.04.06.05.01.02.04.00.0', 4): 88.15632439976874,
('5.05.06.05.01.05.05.09.0', 4): 98.4069022056369,
('5.04.06.05.08.03.05.02.0', 5): 91.80361747402792,
('5.05.05.05.01.05.06.08.0', 1): 92.83741804857122,
('5.05.05.05.01.04.06.07.0', 1): 91.46452042426296,
('5.05.05.05.01.06.06.09.0', 1): 91.80187047665119,
('5.05.07.04.01.05.05.09.0', 1): 90.75743358727908,
('5.05.05.05.01.05.06.010.0', 4): 92.25367551696063,
('5.04.05.05.09.03.05.03.0', 4): 90.19810172375408,
('5.05.05.05.010.05.06.09.0', 2): 92.0457459956382,
('5.05.05.05.010.04.06.07.0', 5): 97.69170499998594,
('5.04.06.05.09.03.06.03.0', 4): 91.95896329716588,
('6.04.06.05.01.01.04.01.0', 2): -14.025097687297247,
('5.05.05.05.01.05.05.09.0', 5): 91.89134040626935,
('5.05.05.05.06.05.06.08.0', 5): 96.8007879150811,
('6.04.06.05.01.03.05.01.0', 5): 89.31222271275425,
('5.05.05.05.01.04.06.06.0', 4): 78.0329390584822,
('6.04.06.05.010.02.05.00.0', 1): 87.1342938966118,
('5.04.06.05.01.03.05.01.0', 4): 89.35065859115987,
('5.04.05.05.08.04.06.04.0', 4): 92.27113663782394,
('5.04.05.05.01.04.06.04.0', 4): 91.28035983476508,
('6.03.06.04.01.05.06.09.0', 5): -12.662126432305499,
('5.06.06.04.01.05.05.09.0', 1): 91.21016731184886,
('5.05.05.05.01.05.05.010.0', 2): 92.3318148216009,
('5.04.06.04.01.06.06.09.0', 1): 90.74317309850602,
('5.05.06.05.01.06.06.09.0', 4): 91.26694531394625,
('6.04.07.04.02.05.06.09.0', 1): 88.50886300180126,
('6.04.06.05.01.02.04.01.0', 3): 87.9451172466112,
('5.04.06.05.01.06.06.09.0', 5): 90.88795110923662,
('5.05.05.05.010.04.06.06.0', 5): 91.03602207674861,
('5.04.05.05.01.04.06.05.0', 3): 97.72112771631718,
('5.05.06.04.01.05.05.010.0', 0): 98.59844272448959,
('5.05.05.05.01.05.06.09.0', 3): 118.36419973912342,
('6.04.06.05.01.03.05.00.0', 2): 88.95688066922293,
('6.04.07.04.01.05.06.09.0', 1): 89.69481153982498,
('5.05.06.04.01.05.05.09.0', 4): 91.07960076454471,
('5.05.06.05.010.05.05.09.0', 0): 91.34805054070965,
('5.04.05.05.01.03.05.03.0', 2): 90.86470830838176
}
"""

class PlotQLearnMatrix(object):
    def __init__(self, real_time=True, plot_values_rt=False):

        self._plot_values_rt = plot_values_rt
        # Set up the Plot system
        self.fig, self.ax = plt.subplots()

        self.state_tags_list = []
        self.state_actions_list = []

        q_learn_data = None
        while q_learn_data is None and not rospy.is_shutdown():
            try:
                q_learn_data = rospy.wait_for_message("/q_learn_matrix", QLearnMatrix, timeout=3.0)
                self.qlearn_matrix = self.convert_qlearn_msgs_to_matrix(q_learn_data)
                self.ax.set_aspect('auto')
                rospy.loginfo("/q_learn_matrix READY==>q_learn_data")
            except:
                rospy.loginfo("Current /q_learn_matrix not ready yet, retrying...")

        if real_time:
            rospy.on_shutdown(self.clean_up_plots)
            rospy.Subscriber("/q_learn_matrix", QLearnMatrix, self.qlearn_callback)

            STATES_length_AXIS = len(self.state_tags_list)
            ACTIONS_length_AXIS = len(self.state_actions_list)

            self.qlearn_matrix = np.zeros((STATES_length_AXIS, ACTIONS_length_AXIS))
            num_elemnts_matrix = self.qlearn_matrix.shape[0] * self.qlearn_matrix.shape[1]
            max_elem = math.pow(10, 6)

            if num_elemnts_matrix > max_elem:
                print ("Warning, The Dimension is too high==>" + str(num_elemnts_matrix) + "> " + str(max_elem))

            self.matrice = self.ax.matshow(self.qlearn_matrix)

            new_cbar = plt.colorbar(self.matrice)
            new_cbar.ax.set_autoscale_on(True)

            ani = animation.FuncAnimation(self.fig, self.update, frames=19, interval=100)
            print "Matrix Ready"
            plt.show()

        else:
            # Start One Image Plot
            # Wait for Matrix generation and shutdown signal
            rospy.loginfo("Plotting Data...")

            self.matrice = self.ax.matshow(self.qlearn_matrix)
            plt.colorbar(self.matrice)

            # We want to show all ticks...
            self.ax.set_xticks(np.arange(len(self.state_actions_list)))
            self.ax.set_yticks(np.arange(len(self.state_tags_list)))
            # ... and label them with the respective list entries
            self.ax.set_xticklabels(self.state_actions_list)
            self.ax.set_yticklabels(self.state_tags_list)

            # Rotate the tick labels and set their alignment.
            plt.setp(self.ax.get_xticklabels(), rotation=45, ha="right",
                     rotation_mode="anchor")

            # Loop over data dimensions and create text annotations.
            for i in range(len(self.state_tags_list)):
                for j in range(len(self.state_actions_list)):
                    text = self.ax.text(j, i, self.qlearn_matrix[i, j],
                                   ha="center", va="center", color="w")

            self.ax.set_title("Q Learn Matrix Actions vs States and rewards value")
            self.ax.set_aspect('auto')
            self.fig.tight_layout()
            plt.show()
            rospy.loginfo("Plotting Data...DONE")

    def update(self,i):
        print ("Update Plot")
        self.matrice.remove()
        self.matrice = self.ax.matshow(self.qlearn_matrix)
        self.ax.set_aspect('auto')

        # We want to show all ticks...
        self.ax.set_xticks(np.arange(len(self.state_actions_list)))
        self.ax.set_yticks(np.arange(len(self.state_tags_list)))
        # ... and label them with the respective list entries
        self.ax.set_xticklabels(self.state_actions_list)
        self.ax.set_yticklabels(self.state_tags_list)

        # Rotate the tick labels and set their alignment.
        plt.setp(self.ax.get_xticklabels(), rotation=45, ha="right",
                 rotation_mode="anchor")

        # Loop over data dimensions and create text annotations.
        # TODO: Find a fix to avoid over writting on top

        if self._plot_values_rt:
            for i in range(len(self.state_tags_list)):
                for j in range(len(self.state_actions_list)):
                    self.text = self.ax.text(j, i, self.qlearn_matrix[i, j],
                                        ha="center", va="center", color="w")







    def qlearn_callback(self,msg):
        self.qlearn_matrix = self.convert_qlearn_msgs_to_matrix(msg)

    def clean_up_plots(self):
        """
        Called when CTRL+C to kill all the plots
        :return:
        """
        rospy.loginfo("Cleanning All the Plots")
        plt.close('all')
        rospy.loginfo("Cleanning All the Plots DONE")

    def convert_qlearn_msgs_to_matrix(self,q_learn_msg):
        """
        Converts a Qlearn Ros Message to a matrix.
        :param q_learn_msg:
        :return:qlearn_matrix
        """
        for qlearn_elem in q_learn_msg.q_learn_matrix:
            q_learn_tag = qlearn_elem.qlearn_point.state_tag.data
            q_learn_action_num = qlearn_elem.qlearn_point.action_number.data


            if not q_learn_tag in self.state_tags_list:
                self.state_tags_list.append(q_learn_tag)

            if not q_learn_action_num in self.state_actions_list:
                self.state_actions_list.append(q_learn_action_num)

        STATES_length_AXIS = len(self.state_tags_list)
        ACTIONS_length_AXIS = len(self.state_actions_list)


        qlearn_matrix = np.zeros((STATES_length_AXIS, ACTIONS_length_AXIS))

        # Fill in the data
        for qlearn_elem in q_learn_msg.q_learn_matrix:
            q_learn_tag = qlearn_elem.qlearn_point.state_tag.data
            q_learn_action_num = qlearn_elem.qlearn_point.action_number.data
            reward_value = qlearn_elem.reward.data

            state_index = self.state_tags_list.index(q_learn_tag)

            action_index = self.state_actions_list.index(q_learn_action_num)

            # We only want two decimals so we just multiply by 100 and convert to integer
            qlearn_matrix[state_index][action_index] = int(reward_value * 100)

        return qlearn_matrix

if __name__ == '__main__':
    rospy.init_node('Q_learn_ploter_node', anonymous=True, log_level=rospy.INFO)
    plotq_object = PlotQLearnMatrix(real_time=False, plot_values_rt=False)