'''
hard_landing_demo_controller.py
'''

from roombasim.ai import Controller

class HardLandingDemoController(Controller):
    '''
    A demonstration of controlled landing while in motion.
    '''

    def setup(self):
        self.landing = False

        # set the initial target
        print('Initiating takeoff...')
        self.task_controller.switch_task(
            'TakeoffTask',
            callback = (lambda a,b: self.move())
        )

    def move(self):
        print('Start movement')
        self.task_controller.switch_task(
            'XYZTranslationTask',
            target = [10,10,2],
            callback = (lambda a,b: self.move())
        )

    def complete(self):
        print('Landed')

    def update(self, delta, elapsed, environment):
        if elapsed > 7000 and self.landing == False:
            print('Land!')
            self.landing = True
            self.task_controller.switch_task(
                'LandTask',
                callback=(lambda a,b: self.complete())
            )
