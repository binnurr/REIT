import os
import rospy
from std_msgs.msg import String
import time
import sys
import csv
import json
import pandas as pd

import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.colors as colors
import numpy as np


class FeedbackAnalyzer:
    def __init__(self, subject_name, session_type, behavioral_feedback):
        rospy.init_node('feedback_analyzer', anonymous=True)
        self.subject_name = subject_name
        self.session_type = session_type
        self.is_behavioral_feedback = behavioral_feedback
        file_p_prefix = os.path.join(os.environ["REIT_HOME"], "BehavioralFeedbackEvaluator/resources/experiment/results")
        self.overall_result_fp = os.path.join(file_p_prefix, subject_name, session_type, "overall_results.csv")
        self.overall_result_img_fp = os.path.join(file_p_prefix, subject_name, session_type, "overall_results.png")
        try:
            os.remove(self.overall_result_img_fp)
        except OSError:
            pass
        os.makedirs(os.path.dirname(self.overall_result_img_fp), exist_ok=True)

        self.robot_feedback_dict = None
        self.feedback_eval_states = None

        rospy.Subscriber('robot_feedback_dict', String, self.feedback_dict_cb, queue_size=1)
        rospy.Subscriber('feedback_stat', String, self.feedback_eval_dict_cb, queue_size=1)

        rospy.Subscriber('is_vis_deneme', String, self.is_start_cb, queue_size=1)

        self.img_ready_pub = rospy.Publisher('feedback_img_ready', String, queue_size=1)

        rospy.spin()

    def is_start_cb(self, msg):
        self.print_img()

    def feedback_dict_cb(self, msg):
        self.robot_feedback_dict = json.loads(str(msg.data))

    def feedback_eval_dict_cb(self, msg):
        self.feedback_eval_states = json.loads(str(msg.data))
        self.print_img()

    def print_img(self):
        # This dictionary defines the colormap
        cdict = {'red': ((0.0, 0.0, 0.0),  # no red at 0
                         (0.5, 1.0, 1.0),  # all channels set to 1.0 at 0.5 to create white
                         (1.0, 0.8, 0.8)),  # set to 0.8 so its not too bright at 1

                 'green': ((0.0, 0.8, 0.8),  # set to 0.8 so its not too bright at 0
                           (0.5, 1.0, 1.0),  # all channels set to 1.0 at 0.5 to create white
                           (1.0, 0.0, 0.0)),  # no green at 1

                 'blue': ((0.0, 0.0, 0.0),  # no blue at 0
                          (0.5, 1.0, 1.0),  # all channels set to 1.0 at 0.5 to create white
                          (1.0, 0.0, 0.0))  # no blue at 1
                 }

        # Create the colormap using the dictionary
        GnRd = colors.LinearSegmentedColormap('GnRd', cdict)

        robot_feedback_dict = self.robot_feedback_dict
        feedback_eval_states = self.feedback_eval_states

        '''
        robot_feedback_dict = {'0': ['No rapport with customer'], '1': ['No rapport with customer'],
                               '2': ['Asking long question'], '3': ['Lack of preparation'],
                               '4': ['Showing lack of preparation'],
                               '5': ['Showing lack of preparation', 'Asking long/complex question'],
                               '6': ['No mistake'], '7': ['Asking technical question'],
                               '8': ['Asking long/complex question', 'Showing lack of preparation'],
                               '9': ['Lack of preparation'], '10': ['Showing lack of preparation'],
                               '11': ['No mistake'], '12': ['Asking vague question'], '13': ['Asking vague question'],
                               '14': ['Asking vague question', 'Asking technical question'],
                               '15': ['Unnatural dialog style'], '16': ['No mistake']}
        feedback_eval_states = {'0': ['No rapport with customer'], '1': ['No mistake'],
                                '2': ['Lack of planning for interview'], '3': ['Showing lack of preparation'],
                                '4': ['Showing lack of preparation'], '5': ['Lack of preparation'],
                                '7': ['Asking technical question', 'Lack of planning for interview'],
                                '8': ['Showing lack of preparation'], '9': ['Influencing customer'],
                                '10': ['Asking vague question'], '12': ['No mistake'], '13': ['No mistake'],
                                '14': ['Showing lack of preparation', 'Not asking about existing system'],
                                '15': ['No mistake']}
        '''

        res_df = pd.DataFrame()
        res_df['session'] = ['after_feedback', 'during_interview']

        total_ques = len(robot_feedback_dict)
        for i in range(total_ques):
            col_name = str(i + 1)
            res_df[col_name] = -3

        for i in range(total_ques):
            col_name = str(i + 1)
            if robot_feedback_dict[str(i)] != ["No mistake"]:
                res_df[col_name].loc[res_df['session'] == 'during_interview'] = 3

        for ind, mist in feedback_eval_states.items():
            if mist != ["No mistake"]:
                col_name = str(int(ind) + 1)
                res_df[col_name].loc[res_df.session == 'after_feedback'] = 3

        res_df = res_df.set_index('session')
        plt.switch_backend('Agg')
        # Visualize the data as heatmap
        plt.rcParams['font.size'] = 15

        num_fig = 2
        height_ratios = [1, 2]
        if self.is_behavioral_feedback:
            num_fig = 3
            height_ratios = [2, 1, 2]
        fig, axis = plt.subplots(num_fig, 1, figsize=(12, 18), gridspec_kw={'height_ratios': height_ratios})

        sns.heatmap(res_df, cmap=GnRd, cbar=False, ax=axis[num_fig-2], linewidths=5, linecolor='black')

        axis[num_fig-2].set_yticks([i for i in range(len(res_df))], [i for i in res_df.index.values])
        axis[num_fig-2].set_yticklabels(['after feedback', 'during interview'], ha='right', rotation=0)
        axis[num_fig-2].set_ylabel('')
        axis[num_fig-2].set_xlabel('Interaction ID')
        axis[num_fig-2].set_title('Correctness of Choices Before and After Feedback per Interaction')

        # plt.show()

        mistake_dict = {}
        mistake_dict['No mistake'] = 'No mistake'
        mistake_dict['No rapport with customer'] = 'No rapport with customer'
        mistake_dict['Asking long question'] = 'Asking long question'
        mistake_dict['Asking long/complex question'] = 'Asking long question'
        mistake_dict['Lack of planning for interview'] = 'Lack of preparation'
        mistake_dict['Lack of preparation'] = 'Lack of preparation'
        mistake_dict['Showing lack of preparation'] = 'Lack of preparation'
        mistake_dict['Asking technical question'] = 'Asking technical question'
        mistake_dict['Influencing customer'] = 'Influencing customer'
        mistake_dict['Asking vague question'] = 'Asking vague question'
        mistake_dict['Asking customer for solution'] = 'Asking customer for solution'
        mistake_dict['Not asking about existing system'] = 'Not asking about existing system'
        mistake_dict['Unnatural dialog style'] = 'Unnatural dialog style'
        mistake_dict['Not identifying stakeholders'] = 'Not identifying stakeholders'
        mistake_dict['Asking unnecessary question'] = 'Asking unnecessary question'
        mistake_dict['Incorrect ending of interview'] = 'Incorrect ending of interview'
        mistake_dict['Did not provide short summary'] = 'Did not provide short summary'

        mistakes = ['No rapport with customer', 'Asking long question', 'Lack of preparation',
                    'Asking technical question', 'Influencing customer',
                    'Asking vague question', 'Asking customer for solution', 'Not asking about existing system',
                    'Unnatural dialog style',
                    'Not identifying stakeholders', 'Asking unnecessary question', 'Incorrect ending of interview',
                    'Did not provide short summary']

        before_feedback_count = []
        after_feedback_count = []

        given_feedback_per_group = {}
        for ind, mist_list in robot_feedback_dict.items():
            for mist in mist_list:
                if mistake_dict[mist] in given_feedback_per_group:
                    given_feedback_per_group[mistake_dict[mist]] = given_feedback_per_group[mistake_dict[mist]] + 1
                else:
                    given_feedback_per_group[mistake_dict[mist]] = 1

        for mistake in mistakes:
            if mistake in given_feedback_per_group:
                before_feedback_count.append(given_feedback_per_group[mistake])
            else:
                before_feedback_count.append(0)

        given_feedback_per_group = {}
        for ind, mist_list in feedback_eval_states.items():
            for mist in mist_list:
                if mistake_dict[mist] in given_feedback_per_group:
                    given_feedback_per_group[mistake_dict[mist]] = given_feedback_per_group[mistake_dict[mist]] + 1
                else:
                    given_feedback_per_group[mistake_dict[mist]] = 1

        for mistake in mistakes:
            if mistake in given_feedback_per_group:
                after_feedback_count.append(given_feedback_per_group[mistake])
            else:
                after_feedback_count.append(0)

        print(before_feedback_count)
        print(after_feedback_count)

        del_index = []
        for i, count in enumerate(before_feedback_count):
            if before_feedback_count[i] == 0 and after_feedback_count[i] == 0:
                del_index.append(i)
        for del_i in reversed(del_index):
            del before_feedback_count[del_i]
            del after_feedback_count[del_i]
            del mistakes[del_i]

        print(before_feedback_count)
        print(after_feedback_count)

        # fig, ax = plt.subplots(1, 1, figsize=(12, 4))
        x_axis = np.arange(len(mistakes))
        axis[num_fig-1].bar(x_axis - 0.2, before_feedback_count, width=0.4, label='during_interview')
        axis[num_fig-1].bar(x_axis + 0.2, after_feedback_count, width=0.4, label='after_feedback')

        print(mistakes)

        try:
            yint = []
            locs = axis[num_fig-1].get_yticks()
            for each in locs:
                yint.append(int(each))
            axis[num_fig-1].set_yticks(yint)
        except:
            pass


        axis[num_fig-1].set_xticks(range(len(mistakes)))
        axis[num_fig-1].set_xticklabels(mistakes, ha='right', rotation=40, wrap=True)
        axis[num_fig-1].set_title('Mistake Counts per Mistake Type')
        axis[num_fig-1].legend()

        if self.is_behavioral_feedback:
            self.read_video_analysis_res()
            x = list(range(1, len(self.arousal_mean_list)+1))
            # create an index for each tick position
            xi = list(range(len(x)))
            y = self.arousal_mean_list
            y_std = self.arousal_std_list

            y1 = self.valence_mean_list
            y1_std = self.valence_std_list

            axis[0].set_ylim(-1, 1)
            # plot the index for the x-values
            axis[0].errorbar(xi, y, y_std, marker='*', markersize=15, linestyle='--', color='b', label='Excitement')
            axis[0].errorbar(xi, y1, y1_std, marker='h',  markersize=15, linestyle='--', color='magenta', label='Positivity')
            '''
            axis[0].axhspan(0.8, 1.0, facecolor='darkgreen', alpha=0.9)
            axis[0].axhspan(0.6, 0.8, facecolor='darkgreen', alpha=0.8)
            axis[0].axhspan(0.4, 0.6, facecolor='darkgreen', alpha=0.7)
            axis[0].axhspan(0.2, 0.4, facecolor='green', alpha=0.5)
            axis[0].axhspan(0.05, 0.2, facecolor='palegreen', alpha=0.6)
            axis[0].axhspan(-0.05, 0.05, facecolor='white', alpha=0.5)
            axis[0].axhspan(-0.8, -1.0, facecolor='darkred', alpha=0.9)
            axis[0].axhspan(-0.6, -0.8, facecolor='darkred', alpha=0.8)
            axis[0].axhspan(-0.4, -0.6, facecolor='darkred', alpha=0.7)
            axis[0].axhspan(-0.2, -0.4, facecolor='red', alpha=0.3)
            axis[0].axhspan(-0.05, -0.2, facecolor='lightcoral', alpha=0.4)
            '''

            axis[0].axhspan(0.85, 1.0, facecolor='darkred', alpha=0.7)
            axis[0].axhspan(0.65, 0.85, facecolor='red', alpha=0.3)
            axis[0].axhspan(0.5, 0.65, facecolor='lightcoral', alpha=0.4)
            axis[0].axhspan(0.45, 0.5, facecolor='white', alpha=0.5)
            axis[0].axhspan(0.25, 0.45, facecolor='palegreen', alpha=0.6)
            axis[0].axhspan(0.0, 0.25, facecolor='darkgreen', alpha=0.8)
            axis[0].axhspan(-0.2, 0.0, facecolor='palegreen', alpha=0.6)
            axis[0].axhspan(-0.2, -0.25, facecolor='white', alpha=0.5)
            axis[0].axhspan(-0.4, -0.25, facecolor='lightcoral', alpha=0.4)
            axis[0].axhspan(-0.6, -0.4, facecolor='red', alpha=0.3)
            axis[0].axhspan(-0.8, -0.6, facecolor='darkred', alpha=0.7)
            axis[0].axhspan(-1.0, -0.8, facecolor='darkred', alpha=0.9)


            axis[0].set_xlabel('Interaction ID')
            axis[0].set_ylabel('')
            axis[0].set_xticks(xi, x)
            axis[0].set_title('Facial Behavior Analysis')
            axis[0].legend()

        plt.tight_layout()
        plt.savefig(self.overall_result_img_fp)
        # plt.show()
        self.img_ready_pub.publish(String('ready'))

    def read_video_analysis_res(self):
        file = open(self.overall_result_fp)
        csv_reader = csv.reader(file)
        self.arousal_mean_list = []
        self.arousal_std_list = []
        self.valence_mean_list = []
        self.valence_std_list = []
        for row in csv_reader:
            self.arousal_mean_list.append(float(row[1]))
            self.arousal_std_list.append(float(row[2]))
            self.valence_mean_list.append(float(row[3]))
            self.valence_std_list.append(float(row[4]))
        file.close()


if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    subject_name = args[1]
    session_type = args[2]
    behavioral_feedback = bool(args[3])
    FeedbackAnalyzer(subject_name, session_type, behavioral_feedback)
