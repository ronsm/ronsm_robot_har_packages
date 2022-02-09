#!/usr/bin/env python3

from log import Log

data = {
    'Other' : ['doing something else', 'other'],
    'Relaxing' : ['relaxing', 'sitting doing nothing', 'pottering about in the garden', 'chilling', 'taking a break', 'sitting down', 'chilling out', 'having a lie down', 'on a break'],
    'Working' : ['working', 'doing work', 'sitting at my desk', 'doing paperwork', 'doing bookkeeping', 'doing some work', 'working at my desk'],
    'Studying' : ['studying', 'studying for an exam', 'studying for a test', 'doing some homework', 'going over my notes', 'reading my study notes'],
    'Sleeping' : ['trying to sleep', 'tossing and turning', 'trying to fall asleep', 'trying to get comfortable', 'going to bed', 'recharging my batteries', 'going to bed', 'taking a nap', 'having a snooze'],
    'LeavingHome' : ['going out', 'going to the post office', 'visiting friends', 'heading out for a while', 'leaving', 'going out to work in the garden', 'going for a walk', 'getting some fresh air', 'going outside', 'leaving the house', 'getting out of the house', 'nipping out', 'popping out'],
    'Bathing' : ['having a bath', 'going for a bath', 'relaxing in bath', 'taking a wash', 'taking a bath', 'washing myself', 'having a wash'],
    'Cooking' : ['cooking', 'making something to eat' 'going to make dinner', 'making dinner', 'making food', 'getting some food', 'making lunch', 'making breakfast', 'cooking breakfast', 'making a snack', ' preparing dinner', 'doing some meal prep'],
    'PreparingDrink' : ['making a drink', 'having a drink of juice', 'making a cup of tea', 'preparing a cup of tea', 'preparing a cup of coffee', 'making tea', 'making coffee', 'grabbing a drink'],
    'Eating' : ['eating food', 'having a meal', 'eating breakfast', 'eating lunch', 'having lunch', 'stuffing my face', 'having dinner', 'munching'],
    'Snacking' : ['grabbing a snack', 'having a treat', 'having a biscuit', 'having some fruit', 'having some scran', 'having a munch', 'having some chocolate', 'eating some sweeties', 'having a snack attack'],
    'WatchingTV' : ['watching TV', 'sitting watching a film', 'sitting watching TV', 'relaxing watching TV', 'watching television', 'watching a programme', 'watching a film', 'watching a movie', 'telly'],
    'UsingComputer' : ['on the computer', 'sitting on my laptop', 'going to use my laptop', 'spending time on the PC', 'using my PC', 'working on my PC', 'on my computer', 'on my laptop'],
    'UsingSmartphone' : ['using smarthone', 'mucking about on my phone', 'making a call', 'looking at my phone', 'browsing on my phone', 'browsing Facebook', 'scrolling Twitter', 'scrolling Instagram', 'scrolling Facebook', 'browsing Instagram', 'texting someone', 'texting'],
    'UsingInternet' : ['on the internet', 'shopping online', 'browsing the internet', 'looking something up on the internet', 'browsing the web', 'looking at something on the internet'],
    'WashingDishes' : ['doing dishes', 'washing dishes', 'doing the washing up', 'have to do the dishes', 'washing the dishes', 'washing up'],
    'Showering' : ['showering', 'taking a shower', 'getting showered', 'having a shower', 'getting showered'],
    'Reading' : ['reading', 'getting into a book', 'reading my kindle', 'reading my book', 'reading a magazine', 'browsing a magazine', 'perusing a newpaper', 'reading a paper', 'read a book', 'reading a book'],
    'DoingLaundry' : ['doing laundry', 'doing a load of washing', 'loading the washing machine', 'doing the laundry', 'doing a washing', 'putting a wash on', 'washing clothes'],
    'Shaving' : ['shaving', 'shaving my legs', 'shaving my armpits', 'having a shave', 'having a trim', 'having a shave'],
    'BrushingTeeth' : ['brushing teeth', 'cleaning my teeth', 'brush my teeth'],
    'TalkingOnPhone' : ['on the phone', 'yapping on the phone', 'on the phone with someone', 'on a call', 'having a phone call', 'talking to someone on the phone', 'talking to my friend on the phone', 'phoning someone'],
    'ListeningToMusic' : ['listening to music', 'enjoying some music', 'putting some tunes on', 'listening to the radio', 'playing a CD', 'listening to a record', 'listening to Spotify', 'got my headphones on'],
    'Cleaning' : ['cleaning', 'gutting the house', 'dusting', 'washing floors', 'doing the housework', 'cleaning up', 'sorting the place out', 'cleaning the house', 'cleaning my room', 'tidying the house', 'cleaning the bathroom', 'vacuuming', 'hoovering up'],
    'Conversing' : ['having a chat', 'chatting to the neighbours', 'talking', 'yapping', 'talking to friends', 'talking to someone', 'talking to my friend', 'having a debate'],
    'HostingGuests' : ['having people over', 'have visitors', 'have friends over', 'friends round', 'hanging out', 'hanging out with friends', 'have a friend round', 'having a party', 'having a dinner party', 'hosting a dinner party'],
    'GettingDressed' : ['getting dressed', 'getting ready', 'putting my clothes on', 'putting clothes on', 'get ready', 'getting undressed', 'getting changed', 'changing clothes', 'putting my outfit on', 'getting dressed up', 'putting my makeup on'],
    'TakingMedication' : ['taking pills', 'taking some pills', 'swallowing some pills', 'taking my tablets', 'taking my pills', 'taking medication', 'taking the pill', 'taking medicine'],
    'Tidying' : ['tidying up', 'doing a quick tidy', 'tidy up', 'getting organised', 'organising the house', 'tidying the house', 'having a clear up', 'taking out the bins'],
    'Toileting' : ['using the toilet', 'on the loo', 'having a wee', 'going to the toilet', 'going to the loo', 'having a piss', 'going to the loo', 'having a poo', 'taking a dump', 'doing a poo', 'having a pee', 'nipping to the loo', 'going for a pee', 'going to the bathroom', 'using the bathroom'],
    'Exercising' : ['exercising', 'walking the dog', 'having a walk', 'working out', 'going to a gym class', 'doing my exercises', 'on the treadmill', 'lifting weights', 'doing a fitness class'],
    'Drinking' : ['having a drink', 'having a cuppa', 'drinking my tea', 'drinking my coffee', 'drink some water', 'taking a drink', 'warming myself up with a hot drink', 'having a drink of water'],
    'Hobby' : ['doing a hobby', 'having some downtime', 'building my jigsaw', 'doing a jigsaw', 'mucking about', 'doing some DIY', 'playing cards', 'playing board games', 'painting', 'playing chess', 'writing', 'pottering about in the garden', 'working in the garden', 'working in the garden'],
}

class SemanticADLs(object):
    def __init__(self):
        self.id = 'semantic_ADLs'

        self.logger = Log(self.id)

        self.logger.log_great('Ready.')

    def get_semantic_ADLs(self):
        return data

    def get_ADL_descriptor(self, key):
        return data[key][0]

    def get_ADL_from_descriptor(self, descriptor):
        for key, value in data.items():
            for v in value:
                if v == descriptor:
                    return key