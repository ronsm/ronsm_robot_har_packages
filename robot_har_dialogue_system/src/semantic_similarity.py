#!/usr/bin/env python3

from log import Log
import spacy
import pprint
import numpy as np

from semantic_ADLs import SemanticADLs

nlp_eng = spacy.load('en_core_web_lg')

SIMILARITY_MARGIN = 0.1

class SemanticSimilarity(object):
    def __init__(self, semantic_ADLs):
        self.id = 'semantic_similarity'

        self.logger = Log(self.id)

        self.semantic_ADLs = semantic_ADLs

        self.labels_dict = self.semantic_ADLs.get_semantic_ADLs()

    def compare_similarity(self, compare, labels=[], compare_all=False):
        follow_up = False
        options = ['']

        if compare_all:
            similarity_scores = self.compute_similarity(compare)
            similarity_scores_sorted, top_label = self.sort_similarity_scores(similarity_scores)
            return follow_up, options, top_label
        else:
            if len(labels) == 2:
                similarity_scores = self.compute_similarity(compare)
                similarity_scores_sorted, top_label = self.sort_similarity_scores(similarity_scores)
                return follow_up, options, top_label
            elif len(labels) == 3:
                similarity_scores = self.compute_similarity(compare)
                similarity_scores_sorted, top_label = self.sort_similarity_scores(similarity_scores)
                follow_up, options = self.evaluate_follow_up(similarity_scores_sorted)
                options = self.get_options_natural_descriptions(options)
                return follow_up, options, top_label
            else:
                self.logger.log_warn('Invalid number of labels provided. Upstream error.')

    def compute_similarity(self, compare):
        all_similarity_scores = {}
        compare = nlp_eng(compare)
        
        for key, value in self.labels_dict.items():
            class_descriptions = []
            similarity_scores = []

            for item in value:
                class_descriptions.append(nlp_eng(item))
            
            for class_description in class_descriptions:
                similarity_score = class_description.similarity(compare)
                similarity_scores.append(similarity_score)

            all_similarity_scores[key] = similarity_scores

        return all_similarity_scores

    def evaluate_follow_up(self, similarity_scores):
        follow_up = False

        key_1 = list(similarity_scores)[0]
        key_2 = list(similarity_scores)[1]

        value_1 = list(similarity_scores.values())[0]
        value_2 = list(similarity_scores.values())[1]

        options = [key_1, key_2]

        margin = value_2 - value_1
        if margin < 0:
            margin = margin * -1.0

        if margin < SIMILARITY_MARGIN:
            follow_up = True

        if follow_up:
            self.logger.log_warn('There will be a follow up question.')

        return follow_up, options

    def sort_similarity_scores(self, similarity_scores):
        similarity_scores_argmax = {}

        for key, value in similarity_scores.items():
            np_array = np.asarray(value)
            argmax = np_array[np_array.argmax()]

            similarity_scores_argmax[key] = argmax

        data_sorted = {k: v for k, v in sorted(similarity_scores_argmax.items(), reverse=True, key=lambda x: x[1])}

        top_label = list(data_sorted)[0]

        print(data_sorted)

        return data_sorted, top_label

    def get_options_natural_descriptions(self, options):
        options_natural_descriptions = []
        for option in options:
            natural_description = self.labels_dict.get(option)
            natural_description = natural_description[0]
            options_natural_descriptions.append(natural_description)
        return options_natural_descriptions