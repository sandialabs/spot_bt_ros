from __future__ import annotations

import py_trees


def generic_post_tick_handler(behavior_tree: py_trees.trees.BehaviourTree):
    """Print a generic ASCII tree after a tick."""
    print(
        py_trees.display.unicode_tree(
            root=behavior_tree.root, show_status=True
        )
    )


def generic_pre_tick_handler(behavior_tree: py_trees.trees.BehaviourTree):
    """Print a generic banner showing the current count."""
    print(f"--------- Run {behavior_tree.count} ---------")


def snapshot_added_post_tick_handler(
    snapshot_visitor: py_trees.visitors.SnapshotVisitor,
    behavior_tree: py_trees.trees.BehaviourTree,
):
    """Print an ASCII tree with the current snapshot status."""
    print(
        py_trees.display.unicode_tree(
            root=behavior_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited,
        )
    )
