
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
from sklearn.metrics import f1_score
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score
import statistics

import random
from numpy.random import randint


class EmotionFuzzy:

    def __init__(self):
        self.no_repetition = 3
        self.no_classes = 5
        self.no_image_per_test = 70

        self.accuracy_score_history = []
        self.overall_performance_history = []
        self.learning_speed_history = []

        self.hope_history = []
        self.joy_history = []
        self.define_fuzzy_system()

    def draw_random_counts(self):
        sum = 0
        result = []
        for c in range(self.no_classes-1):
            temp = random.randint(0, self.no_image_per_test-sum)
            result.append(temp)
            sum += temp
        temp = self.no_image_per_test-sum
        result.append(temp)
        return result

    def draw_random_labels(self):
        return randint(0, self.no_classes, self.no_image_per_test)

    def define_fuzzy_system(self):
        self.overall_performance = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'overall_performance')
        self.last_prediction = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'last_prediction')
        self.learning_speed = ctrl.Antecedent(np.arange(-1, 1.01, 0.01), 'learning_speed')

        self.hope_level = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'hope_level')

        self.hope = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'hope')
        self.joy = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'joy')

        self.overall_performance['low'] = fuzz.gaussmf(self.overall_performance.universe, 0, 0.25)
        self.overall_performance['medium'] = fuzz.gaussmf(self.overall_performance.universe, 0.5, 0.2)
        self.overall_performance['high'] = fuzz.gaussmf(self.overall_performance.universe, 1.0, 0.25)

        self.last_prediction['highly_not_accurate'] = fuzz.zmf(self.last_prediction.universe, 0, 1.0/self.no_classes)
        self.last_prediction['slightly_not_accurate'] = fuzz.smf_end(self.last_prediction.universe, 0, 1.0/self.no_classes + 0.05, 1.0/self.no_classes)
        self.last_prediction['slightly_accurate'] = fuzz.zmf_start(self.last_prediction.universe, 1.0/self.no_classes -0.05, 1, 1.0/self.no_classes)
        self.last_prediction['highly_accurate'] = fuzz.smf(self.last_prediction.universe, 1.0/self.no_classes, 1)

        self.learning_speed['low'] = fuzz.trimf(self.learning_speed.universe, [-1, -0.63, -0.33])
        self.learning_speed['medium'] = fuzz.trimf(self.learning_speed.universe, [-0.66, 0, 0.66])
        self.learning_speed['high'] = fuzz.trimf(self.learning_speed.universe, [0.33, 0.66, 1])

        self.hope_level['low'] = fuzz.trimf(self.hope_level.universe, [0, 0, 0.5])
        self.hope_level['medium'] = fuzz.trimf(self.hope_level.universe, [0, 0.5, 1])
        self.hope_level['high'] = fuzz.trimf(self.hope_level.universe, [0.5, 1, 1])

        self.hope['low'] = fuzz.trimf(self.hope.universe, [0, 0, 0.5])
        self.hope['medium'] = fuzz.trimf(self.hope.universe, [0, 0.5, 1])
        self.hope['high'] = fuzz.trimf(self.hope.universe, [0.5, 1, 1])

        self.joy['low'] = fuzz.trimf(self.joy.universe, [0, 0, 0.5])
        self.joy['medium'] = fuzz.trimf(self.joy.universe, [0, 0.5, 1])
        self.joy['high'] = fuzz.trimf(self.joy.universe, [0.5, 1, 1])

        self.hope_rule1 = ctrl.Rule(self.overall_performance['low'] & (self.learning_speed['low'] | self.learning_speed['medium']), self.hope['low'])
        self.hope_rule2 = ctrl.Rule(self.overall_performance['medium'] & self.learning_speed['medium'], self.hope['low'])
        self.hope_rule3 = ctrl.Rule(self.overall_performance['medium'], self.hope['medium'])
        self.hope_rule4 = ctrl.Rule(self.overall_performance['high'] | self.learning_speed['high'], self.hope['high'])
        #self.hope_rule1.view()

        self.hope_ctrl = ctrl.ControlSystem([self.hope_rule1, self.hope_rule2, self.hope_rule3,self. hope_rule4])
        self.hope_system = ctrl.ControlSystemSimulation(self.hope_ctrl)

        self.joy_rule1 = ctrl.Rule((self.hope_level['low'] | self.hope_level['medium']) & self.last_prediction['highly_not_accurate'], self.joy['low'])
        self.joy_rule2 = ctrl.Rule(self.hope_level['medium'] & (self.last_prediction['slightly_not_accurate'] | self.last_prediction['slightly_accurate']), self.joy['low'])
        self.joy_rule3 = ctrl.Rule(self.hope_level['low'] & self.last_prediction['slightly_accurate'], self.joy['medium'])
        self.joy_rule4 = ctrl.Rule(self.last_prediction['highly_accurate'], self.joy['high'])

        self.joy_ctrl = ctrl.ControlSystem([self.joy_rule1, self.joy_rule2, self.joy_rule3, self.joy_rule4])
        self.joy_system = ctrl.ControlSystemSimulation(self.joy_ctrl)

    def visualize(self):
        self.overall_performance.view()
        self.last_prediction.view()
        self.learning_speed.view()
        self.hope_level.view()
        self.hope.view()
        self.joy.view()
        self.hope.view(sim=self.hope_system)
        self.joy.view(sim=self.joy_system)
        plt.show()

    def plot_system(self):
        fig, ax = plt.subplots()
        ax.plot(self.hope_history, 'rs', label='hope')
        ax.plot(self.joy_history, 'bo', label='joy')
        ax.plot(self.accuracy_score_history, 'ko-', label='accuracy')
        ax.plot(self.overall_performance_history, 'm', label='overall\nperformance')
        ax.plot(self.learning_speed_history, 'g.-', label='learning speed')
        plt.yticks(np.arange(-1, 1.2, 0.2))
        plt.xticks(np.arange(0, len(self.joy_history) + 1, 1))
        #legend = ax.legend(loc='upper right', shadow=False, fontsize='small')
        for r in range(self.no_repetition - 1):
            plt.axvline(x=(r + 1) * self.no_classes - 1 + 0.5, linestyle=':', linewidth=1.5, color='yellow')
        plt.text(1, 1, "episode0", fontsize=10)
        plt.text(6, 1, "episode1", fontsize=10)
        plt.text(11, 1, "episode2", fontsize=10)

        box = ax.get_position()
        ax.set_position([box.x0, box.y0, box.width * 0.85, box.height])
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), shadow=False, fontsize='small')
        plt.show()

    def compute_emotion(self, last_accuracy_score):
        self.accuracy_score_history.append(last_accuracy_score)
        last_overall_performance = statistics.mean(self.accuracy_score_history)
        self.hope_system.input['overall_performance'] = last_overall_performance
        self.overall_performance_history.append(last_overall_performance)
        if len(self.accuracy_score_history) > 1:
            last_learning_speed = self.accuracy_score_history[-1] - self.accuracy_score_history[-2]
        else:
            last_learning_speed = 1
        self.hope_system.input['learning_speed'] = last_learning_speed
        self.learning_speed_history.append(last_learning_speed)
        self.hope_system.compute()
        last_hope = self.hope_system.output['hope']
        self.hope_history.append(last_hope)
        print("Last Hope: " + str(last_hope))
        self.joy_system.input['hope_level'] = last_hope
        self.joy_system.input['last_prediction'] = last_accuracy_score
        self.joy_system.compute()
        last_joy = self.joy_system.output['joy']
        self.joy_history.append(last_joy)
        print("Last Joy: " + str(last_joy))
        return last_hope, last_joy

    def run_simulation(self):
        predicted_labels = []
        ground_t_labels = []
        simulated_score = [0.3, 0.5, 0.4, 0.1, 0.21, 0.6, 0.8, 0.6, 0.5, 0.5, 0.7, 0.85, 0.9, 0.7, 0.55]
        #simulated_score = [0.21, 0.18, 0.23, 0.21, 0.25, 0.5, 0.53, 0.55, 0.58, 0.5, 0.8, 0.85, 0.83, 0.85, 0.9]
        #simulated_score = [0.9, 0.85, 0.83, 0.87, 0.8, 0.53, 0.5, 0.58, 0.55, 0.5, 0.22, 0.21, 0.23, 0.18, 0.21]
        counter = 0
        episode_accuracy = []
        for r in range(self.no_repetition):
            episode_accuracy = []
            for c in range(self.no_classes):
                #ground_t_labels = [c for i in range(self.no_image_per_test)]
                #last_predicted_labels = draw_random_labels().tolist()
                #predicted_labels += last_predicted_labels
                #last_accuracy_score = accuracy_score(ground_t_labels, last_predicted_labels)
                last_accuracy_score = random.random()
                #last_accuracy_score = simulated_score[counter]
                counter += 1
                self.accuracy_score_history.append(last_accuracy_score)
                episode_accuracy.append(last_accuracy_score)
                #print(confusion_matrix(ground_t_labels, predicted_labels))
                last_overall_performance = statistics.mean(self.accuracy_score_history)
                last_overall_performance = statistics.mean(episode_accuracy)
                self.hope_system.input['overall_performance'] = last_overall_performance
                self.overall_performance_history.append(last_overall_performance)
                if len(self.accuracy_score_history) > 1:
                    last_learning_speed = self.accuracy_score_history[-1] - self.accuracy_score_history[-2]
                else:
                    last_learning_speed = 0
                self.hope_system.input['learning_speed'] = last_learning_speed
                self.learning_speed_history.append(last_learning_speed)
                self.hope_system.compute()
                print(counter)
                last_hope = self.hope_system.output['hope']
                self.hope_history.append(last_hope)
                print("Last Hope: " + str(last_hope))
                self.joy_system.input['hope_level'] = last_hope
                self.joy_system.input['last_prediction'] = last_accuracy_score
                self.joy_system.compute()
                last_joy = self.joy_system.output['joy']
                self.joy_history.append(last_joy)
                print("Last Joy: " + str(last_joy))


if __name__ == '__main__':
    fuzzy_emotion_system = EmotionFuzzy()
    fuzzy_emotion_system.run_simulation()
    fuzzy_emotion_system.plot_system()
    #fuzzy_emotion_system.visualize()


