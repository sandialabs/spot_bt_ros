from __future__ import annotations

from py_trees.display import unicode_tree
from py_trees.trees import BehaviourTree
from py_trees.visitors import SnapshotVisitor


def generic_post_tick_handler(behavior_tree: BehaviourTree):
    """Print a generic ASCII tree after a tick."""
    print(unicode_tree(root=behavior_tree.root, show_status=True))


def generic_pre_tick_handler(behavior_tree: BehaviourTree):
    """Print a generic banner showing the current count."""
    print(f"--------- Run {behavior_tree.count} ---------")


def snapshot_added_post_tick_handler(
    snapshot_visitor: SnapshotVisitor, behavior_tree: BehaviourTree
):
    """Print an ASCII tree with the current snapshot status."""
    print(
        unicode_tree(
            root=behavior_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited,
        )
    )
