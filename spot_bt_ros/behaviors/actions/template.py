from __future__ import annotations

import random

import py_trees


class Foo(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        """
        Minimal one-time initialisation. A good rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        super().__init__(name)

    def setup(self, **kwargs):
        """
        When is this called?
          This function should be either manually called by your program
          to setup this behaviour alone, or more commonly, via method
          `~py_trees.behaviour.Behaviour.setup_with_descendants`
          or method `~py_trees.trees.BehaviourTree.setup`, both of which
          will iterate over this behaviour, it's children (it's children's
          children ...) calling method `~py_trees.behaviour.Behaviour.setup`
          on each in turn.

          If you have vital initialisation necessary to the success
          execution of your behaviour, put a guard in your
          `~py_trees.behaviour.Behaviour.initialise` method
          to protect against entry without having been setup.

        What to do here?
          Delayed one-time initialisation that would otherwise interfere
          with offline rendering of this behaviour in a tree to dot graph
          or validation of the behaviour's configuration.

          Good examples include:

          - Hardware or driver initialisation
          - Middleware initialisation (e.g. ROS pubs/subs/services)
          - A parallel checking for a valid policy configuration after
            children have been added or removed
        """
        self.logger.debug(f"  {self.name} [Foo::setup()]")

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        self.logger.debug(f"  {self.name} [Foo::initialise()]")

    def update(self) -> py_trees.common.Status:
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        """
        self.logger.debug(f"  {self.name} [Foo::update()]")
        ready_to_make_a_decision = random.choice([True, False])
        decision = random.choice([True, False])
        if not ready_to_make_a_decision:
            return py_trees.common.Status.RUNNING
        elif decision:
            self.feedback_message = "We are not bar!"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = "Uh oh"
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug(
            f" {self.name} [Foo::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
