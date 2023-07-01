#!/usr/bin/env python
# coding: utf-8

import random
import os
import re
import codecs
import matplotlib.pyplot as plt
import numpy as np
import pyvis.network
import json

try:
    from pathlib import Path
except ImportError:
    from pathlib2 import Path

dirname = os.path.dirname(__file__)
mistakes_file = os.path.join(dirname, "mistakes.txt")
mistakes_new_file = os.path.join(dirname, "mistakes_new.txt")


class Node:
    def __init__(self, node_id, text):
        self.node_id = node_id
        self.text = text
        self.edges = []

    def add_edge(self, edge):
        self.edges.append(edge)


class Edge:
    def __init__(self, from_n, to_n, text, mistake_ids, mistake_descs):
        self.text = text
        self.from_n = from_n
        self.to_n = to_n
        self.mistake_ids = mistake_ids
        self.mistake_descs = mistake_descs


class Scenario:
    def __init__(self, start_node):
        self.nodes = {}
        self.start_n = start_node
        self.curr_n = start_node
        self.traversed_edges = []
        self.traversed_mistakes = []
        self.correct_edges = []
        self.all_possible_edges = []

    def add_node(self, node_id, text):
        node = Node(node_id, text)
        self.nodes[node.node_id] = node

    def add_edge(self, from_n, to_n, text, mistake_ids, mistake_descs):
        edge = Edge(from_n, to_n, text, mistake_ids, mistake_descs)
        self.nodes[from_n].add_edge(edge)

    def traverse(self, edge_sel_list, from_start):
        if from_start:
            self.curr_n = self.start_n
        for edge_sel in edge_sel_list:
            print('robot speech: ', self.nodes[self.curr_n].text)
            for i, edge in enumerate(self.nodes[self.curr_n].edges):
                if sum(edge.mistake_ids) == 0:
                    print('correct selection: ', i, ' ', edge.text)
                    break
            edge = self.nodes[self.curr_n].edges[edge_sel]
            self.curr_n = edge.to_n
            print('subject selection: ', edge_sel, ' ', edge.text)
            print('subject mistake: ', edge.mistake_descs)
        return self.nodes[self.curr_n].text

    def traverse_once(self, edge_sel):
        self.traversed_edges.append(edge_sel)
        correct_choice = False

        possible_edges = []
        for i, edge in enumerate(self.nodes[self.curr_n].edges):
            possible_edges.append(edge.mistake_descs)
        self.all_possible_edges.append(possible_edges)

        for i, edge in enumerate(self.nodes[self.curr_n].edges):
            if sum(edge.mistake_ids) == 0:
                correct_choice = True
                print('correct selection: ', i, ' ', edge.text)
                self.correct_edges.append(i)
                break
        if not correct_choice:
            self.correct_edges.append(-1)
        edge = self.nodes[self.curr_n].edges[edge_sel]
        self.traversed_mistakes.append(edge.mistake_descs)
        self.curr_n = edge.to_n
        print('subject selection: ', edge_sel, ' ', edge.text)
        print('subject mistake: ', edge.mistake_descs)
        return self.nodes[self.curr_n].text

    def get_options(self):
        edge_text = []
        for i, edge in enumerate(self.nodes[self.curr_n].edges):
            edge_text.append(edge.text)
        return edge_text


class ScenarioParser:
    def __init__(self, scenario_id):
        self.scenario_id = scenario_id
        if scenario_id == "1":
            scenario = "ski_resort"
            self.html_fp = os.path.join(dirname, "resources", scenario, "Interview_Ski_Resort_Owner_v2.html")
        else:
            scenario = "research_tool"
            self.html_fp = os.path.join(dirname, "resources", scenario, "InterviewResearchTool.html")

        self.output_path = os.path.join(dirname, "output", scenario)
        Path(self.output_path).mkdir(exist_ok=True, parents=True)
        self.nodes_file = os.path.join(self.output_path, "nodes.txt")
        self.edges_file = os.path.join(self.output_path, "edges.txt")
        self.graph_file = os.path.join(self.output_path, "graph.html")

    def linewrapper(self, text, wrap_length):
        words = text.split()
        output = ""
        for i in range(0, len(words)):
            output += '%s ' % words[i]
            if i != 0 and i % (wrap_length - 1) == 0:
                output += " </br> "
        return output

    def generate_random_color(self, mix):
        red = random.randint(0, 255)
        green = random.randint(0, 255)
        blue = random.randint(0, 255)

        if mix:
            red = (red + mix[0]) // 2
            green = (green + mix[1]) // 2
            blue = (blue + mix[2]) // 2

        color = '#%02x%02x%02x' % (red, green, blue)
        return color

    def parse_scenario(self):
        mistake_to_class = {}
        mistake_to_color = {}
        f_mistakes = open(mistakes_file, "r")
        f_mistakes_new = open(mistakes_new_file, "r")
        mistake_id_to_desc = {}
        for line in f_mistakes.readlines():
            mistake_id, mistake_text, mistake_color = line.split(',')
            mistake_to_class[mistake_text] = mistake_id
            mistake_to_color[mistake_id] = mistake_color
            if mistake_id not in mistake_id_to_desc:
                mistake_id_to_desc[mistake_id] = mistake_text
        mistake_usage = dict.fromkeys(range(len(set(mistake_to_class.values()))), 0)

        mistake_legend = "\n".join("{!r}: {!r}".format(k, v) for k, v in mistake_id_to_desc.items())

        f = codecs.open(self.html_fp, 'r')
        html_text = f.read()

        f_edges = open(self.edges_file, "w")
        f_nodes = open(self.nodes_file, "w")

        html_text = ''.join(html_text.splitlines())

        pattern = re.compile(r'<tw-passagedata(.*?)tw-passagedata>')
        for pttrn in re.finditer(pattern, html_text):
            passage = pttrn.group(1)
            node_name = re.search(r'name="(.*?)".*?tag', passage).group(1)
            node_text_search = re.search(r'The customer responds:(.*?)quot;(.*?)&quot', passage)
            if node_text_search:
                node_text = node_text_search.group(2).replace('&#39;', '\'')
            else:
                node_text = "Greetings and introduction"
            edges = re.finditer(
                r'click: \?(.*?)\)(.*?)&quot;(.*?)&quot(.*?)&quot;(.*?)&quot(.*?)goto: &quot;(.*?)&quot',
                passage)
            for edge in edges:
                goto = edge.group(7)
                mistake = edge.group(3)
                if mistake == '':
                    mistake = 'No mistake'
                mistake = mistake.replace('.', '')
                mistake = mistake.replace('[', '')
                mistake = mistake.replace(']', '')
                mistake = mistake.replace('the ', '')
                mistake = mistake.replace('dialogue', 'dialog')
                mistake = mistake.replace('questions', 'question')
                mistake = mistake.replace('solutions', 'solution')
                mistake_list = mistake.split(' and ')
                mistake_id_list = []
                for mistake in mistake_list:
                    if mistake in mistake_to_class.keys():
                        mistake_id = mistake_to_class[mistake]
                        mistake_id_list.append(mistake_id)
                        mistake_usage[int(mistake_id)] += 1
                    else:
                        print(mistake)
                edge_text = edge.group(5).replace('&#39;', '\'')
                f_edges.write(node_name + ',' + goto + ',' + '&'.join(mistake_id_list) + ',' + edge_text + '\n')
            f_nodes.write(node_name + ',' + node_text + '\n')

        fig, ax = plt.subplots()
        for i, v in enumerate(mistake_usage.values()):
            ax.text(i - 0.25, v + 2, str(v), color='blue', fontweight='bold')
        plt.bar(mistake_usage.keys(), mistake_usage.values(), color='g')
        plt.xticks(np.arange(0, 14, 1))
        plt.yticks(np.arange(0, 55, 5))
        text_pos_x = 1
        text_pos_y = 0.1
        plt.text(text_pos_x, text_pos_y, mistake_legend, fontsize=14,
                 transform=plt.gcf().transFigure, color='green')
        plt.savefig(os.path.join(self.output_path, "mistakes_hist.png"), bbox_inches="tight")

        f_edges.close()
        f_nodes.close()

    def init_scenario_graph(self):
        self.parse_scenario()
        scenario = Scenario('Start')
        pyvis_graph = pyvis.network.Network(height="1000px", width="1000px", notebook=True, directed=True)
        pyvis_graph.set_edge_smooth('dynamic')
        # pyvis_graph.set_options(json.dumps(var_options))

        mistake_desc = []
        mistake_colors = []
        with open(mistakes_new_file, 'r') as file:
            for line in file:
                mistake_id, mistake_text, mistake_color = line.split(',')
                mistake_desc.append(mistake_text)
                mistake_colors.append(mistake_color)
                pyvis_graph.add_node(mistake_text, color=mistake_color, size=12)

        with open(self.nodes_file, 'r') as file:
            for line in file:
                node_id, node_text = line.split(',', 1)
                pyvis_graph.add_node(node_id, title=self.linewrapper(node_text, 10), color="grey", size=12)
                scenario.add_node(node_id, node_text)

        edge_colors = ['red', 'green', 'blue']
        with open(self.edges_file, 'r') as file:
            lines = file.readlines()
            for i in range(0, len(lines), 3):
                for edge_i in [0, 1, 2]:
                    from_node_id, to_node_id, mistake, edge_text = lines[i + edge_i].split(',', 3)
                    mistakes = mistake.split('&')
                    edge_color = edge_colors[edge_i]
                    edge_color = mistake_colors[int(mistakes[0])]
                    edge_title = self.linewrapper(edge_text, 10) + "</br>"
                    mistake_descs = []
                    mistake_ids = []
                    for mistake in mistakes:
                        edge_title += "</br> " + mistake_desc[int(mistake)]
                        mistake_descs.append(mistake_desc[int(mistake)])
                        mistake_ids.append(int(mistake))
                    pyvis_graph.add_edge(from_node_id, to_node_id, title=edge_title, color=edge_color)
                    scenario.add_edge(from_node_id, to_node_id, edge_text, mistake_ids, mistake_descs)

        # pyvis_graph.force_atlas_2based()
        pyvis_graph.show_buttons(filter_=True)
        pyvis_graph.show(self.graph_file)
        return scenario


if __name__ == "__main__":
    scenario_parser = ScenarioParser("1")
    scenario = scenario_parser.init_scenario_graph()

