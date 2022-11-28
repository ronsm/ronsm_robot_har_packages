    # insert at line 316
    if prediction == None:
        self.estimate_state_unbounded()
    else:
        self.estimate_state_bounded(prediction)
   
    # insert at line 400
    def estimate_state_bounded(self, prediction):
        # s1
        s1_candidates = []
        if len(self.s1) > 0:
            chain = self.markov_chains_states[prediction]
            for entry in chain:
                if collections.Counter(self.s1[-1].state) == collections.Counter(entry.state):
                    if self.s1[-1].state == entry.state:
                        s1_candidates.append((CandidateState(prediction, entry, 'exact', {})))
                    else:(CandidateState(prediction, entry, 'approx', {}))

        if len(s1_candidates) > 0:
            if VERBOSE:
                self.logger.log_mini_header('Estimated State (S1) (Bounded)')
                log = str(s1_candidates)
                self.logger.log(log)
            self.select_state(s1_candidates, segment=1)
            self.state_estimate_success_s1 = True
        else:
            if VERBOSE:
                self.logger.log_mini_header('Estimated State (S1) (Bounded)')
                log = 'Unable to find any matching candidates.'
                self.logger.log_warn(log)
            self.state_estimate_success_s1 = False

        # s2
        if self.s2_active:
            s2_candidates = []
            if len(self.s2) > 0:
                chain = self.markov_chains_states[prediction]
                for entry in chain:
                    if collections.Counter(self.s2[-1].state) == collections.Counter(entry.state):
                        if self.s2[-1].state == entry.state:
                            s2_candidates.append((CandidateState(prediction, entry, 'exact', {})))
                        else:
                            s2_candidates.append((CandidateState(prediction, entry, 'approx', {})))

            if len(s2_candidates) > 0:
                if VERBOSE:
                    self.logger.log_mini_header('Estimated State (S2) (Bounded)')
                    log = str(s2_candidates)
                    self.logger.log(log)
                self.select_state(s2_candidates, segment=2)
                self.state_estimate_success_s2 = True
            else:
                if VERBOSE:
                    self.logger.log_mini_header('Estimated State (S2) (Bounded)')
                    log = 'Unable to find any matching candidates.'
                    self.logger.log_warn(log)
                self.state_estimate_success_s2 = False