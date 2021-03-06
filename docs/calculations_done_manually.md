# Manually Performing Calculations

## Example - Simple Complexities: A-Star for Flat Topped Odd Column Shifted Up <a name="example1"></a>

We wish to move from node `S - (0,0)` to node `E - (4,3)`. Each node has a weight `W`. This is a very simple example as we shall treat the complexity between of node as the same: 2 units (this is effectively using the regular A-Star algorithm where the distance between each point is the same).

```txt
             _______           _______
            /       \         /       \
    _______/  (1,3)  \_______/  (3,3)  \_______
   /       \   W:3   /       \   W:3   /   E   \
  /  (0,3)  \_______/  (2,3)  \_______/  (4,3)  \
  \   W:5   /       \   W:3   /       \   W:3   /
   \_______/  (1,2)  \_______/  (3,2)  \_______/
   /       \   W:3   /       \   W:1   /       \
  /  (0,2)  \_______/  (2,2)  \_______/  (4,2)  \
  \   W:3   /       \   W:3   /       \   W:2   /
   \_______/  (1,1)  \_______/  (3,1)  \_______/
   /       \   W:4   /       \   W:7   /       \
  /  (0,1)  \_______/  (2,1)  \_______/  (4,1)  \
  \   W:7   /       \   W:3   /       \   W:3   /
   \_______/  (1,0)  \_______/  (3,0)  \_______/
   /   S   \   W:4   /       \   W:3   /       \
  /  (0,0)  \_______/  (2,0)  \_______/  (4,0)  \
  \   W:6   /       \   W:5   /       \   W:3   /
   \_______/         \_______/         \_______/
```

Calculation form:

* A-Star =  `(0.5 * current_node_complexity) + (0.5 * neighbour_node_complexity) + previous_node_complexities + weight`
* As we're assuming uniform complexity of 2: A-Star = `2 + previous_node_complexities + weight`

However for the first node we are not traversing any complexity so its score is simply its weight.

To begin we establish a data set which will contain the A-Star score of every node we process, we begin by adding the starting nodes A-Star score:

| A-Star | Node |
| ------ | ---- |
| 6 | (0,0) |

Next we expand our starting node (starting North moving clock-wise) and place the discovered nodes (moveable paths) into a queue and calculate thier scores, we sort the queue in A-Star descending order:

* For node `(0,1)`, A-Star: `2 + 7 = 9`
* For node `(1,0)`, A-Star `2 + 4 = 6`

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 6 | (1,0) | (0,0) | 2 |
| 9 | (0,1) | (0,0) | 2 |

And record each unique node in the data set:

| A-Star | Node |
| ------ | ---- |
| 6 | (0,0) |
| 6 | (1,0) |
| 9 | (0,1) |

We then expand from our queue, first for (1,0), we call this QUEUE-A for illustrative purposes:

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 7 | (2,1) | (1,0),(0,0) | 4 |
| 8 | (1,1) | (1,0),(0,0) | 4 |
| 9 | (2,0) | (1,0),(0,0) | 4 |
| 11 | (0,1) | (1,0),(0,0) | 4 |

Then for (0,1), we call this QUEUE-B:

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 7 | (0,2) | (0,1),(0,0) | 4 |
| 8 | (1,1) | (0,1),(0,0) | 4 |
| 8 | (1,0) | (0,1),(0,0) | 4 |

By comparing QUEUE-A and QUEUE-B you'll notice a duplicate `Current node` of `(1,1)`, we actually expand nodes into a single queue one at a time but these temporary queues can illustarte how duplcaites are handled and this is why we maintain a data set of each node score.

A duplicate indictaes we have discovered two paths that lead to the same point. This means we can get rid of the least efficient one, or in this case if they are the same we'll just remove the last one. We do this by comparing the duplicates score to the data set, if the current node being processed has a better A-Star score we overwrite the record in the data set and remove the older inefficient path from the queue. Conversely if the node we've just processed is worse we just remove the worse path from the queue and leave the data set as is.

Our data set is now:

| A-Star | Node |
| ------ | ---- |
| 6 | (0,0) |
| 6 | (1,0) |
| 7 | (2,1) |
| 7 | (0,2) |
| 8 | (1,1) |
| 8 | (1,0) |
| 9 | (0,1) |
| 11 | (0,1) |

And the actual queue with the duplciate route removed (note we removed old processed paths from the queue otherwise their A-Star scores would cause it to infinitely process the starting nodes):

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 7 | (2,1) | (1,0), (0,0) | 4 |
| 7 | (0,2) | (0,1), (0,0) | 4 |
| 8 | (1,1) | (1,0), (0,0) | 4 |
| 8 | (1,0) | (0,1), (0,0) | 4 |
| 9 | (2,0) | (1,0), (0,0) | 4 |
| 11 | (0,1) | (1,0), (0,0) | 4 |

And once again we'll expand the node with the best A-Star score and add them into the queue, removing that processed path and sort it:

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 6 | (2,2) | (2,1), (1,0), (0,0) | 6 |
| 7 | (0,2) | (0,1), (0,0) | 4 |
| 8 | (1,1) | (1,0), (0,0) | 4 |
| 8 | (1,0) | (0,1), (0,0) | 4 |
| 9 | (3,0) | (2,1), (1,0), (0,0) | 6 |
| 9 | (2,0) | (1,0), (0,0) | 4 |
| 11 | (0,1) | (1,0), (0,0) | 4 |
| 13 | (3,2) | (2,1), (1,0),(0,0) | 6 |

| A-Star | Node |
| ------ | ---- |
| 6 | (0,0) |
| 6 | (1,0) |
| 6 | (2,2) |
| 7 | (2,1) |
| 7 | (0,2) |
| 8 | (1,1) |
| 8 | (1,0) |
| 9 | (2,2) |
| 9 | (0,1) |
| 9 | (3,0) |
| 11 | (0,1) |
| 13 | (3,2) |

You may of noticed that by processing `(2,1)` we discovered 6 possible directions for movement, once again this is where the data set comes in. Three of those paths were heading 'backwards' away from the end point meaning they had higher A-Star scores. Comparing those 'backwards' scores to the data set means we simply discard them as more efficient paths have already been discovered.

Processing the best path in the queue again:

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 7 | (0,2) | (0,1), (0,0) | 4 |
| 8 | (1,1) | (1,0), (0,0) | 4 |
| 8 | (1,0) | (0,1), (0,0) | 4 |
| 9 | (3,0) | (2,1), (1,0), (0,0) | 6 |
| 9 | (2,0) | (1,0), (0,0) | 4 |
| 9 | (3,2) | (2,2), (2,1), (1,0), (0,0) | 8 |
| 11 | (0,1) | (1,0), (0,0) | 4 |
| 11 | (1,2) | (2,2), (2,1), (1,0), (0,0) | 8 |
| 11 | (2,3) | (2,2), (2,1), (1,0), (0,0) | 8 |
| 15 | (3,1) | (2,2), (2,1), (1,0), (0,0) | 8 |

| A-Star | Node |
| ------ | ---- |
| 6 | (0,0) |
| 6 | (1,0) |
| 6 | (2,2) |
| 7 | (2,1) |
| 7 | (0,2) |
| 8 | (1,1) |
| 8 | (1,0) |
| 9 | (2,0) |
| 9 | (3,2) |
| 9 | (0,1) |
| 9 | (3,0) |
| 11 | (0,1) |
| 11 | (1,2) |
| 11 | (2,3) |
| 15 | (3,1) |

And again:

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 8 | (1,1) | (1,0), (0,0) | 4 |
| 8 | (1,0) | (0,1), (0,0) | 4 |
| 9 | (3,0) | (2,1), (1,0),(0,0) | 6 |
| 9 | (2,0) | (1,0), (0,0) | 4 |
| 9 | (3,2) | (2,2), (2,1), (1,0), (0,0) | 8 |
| 9 | (1,2) | (0,2), (0,1), (0,0) | 6 |
| 11 | (0,3) | (0,2), (0,1), (0,0) | 6 |
| 11 | (0,1) | (1,0), (0,0) | 4 |
| 11 | (2,3) | (2,2), (2,1), (1,0), (0,0) | 8 |
| 15 | (3,1) | (2,2), (2,1), (1,0), (0,0) | 8 |

| A-Star | Node |
| ------ | ---- |
| 6 | (0,0) |
| 6 | (1,0) |
| 6 | (2,2) |
| 7 | (2,1) |
| 7 | (0,2) |
| 8 | (1,1) |
| 8 | (1,0) |
| 9 | (2,0) |
| 9 | (3,2) |
| 9 | (0,1) |
| 9 | (3,0) |
| 9 | (1,2) |
| 11 | (0,1) |
| 11 | (2,3) |
| 15 | (3,1) |

And again (in fact the top two scores don't determine any new routes more efficient than ones already discoverd so we'll remove them and process the third instead):

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 9 | (2,0) | (1,0), (0,0) | 4 |
| 9 | (3,2) | (2,2), (2,1), (1,0), (0,0) | 8 |
| 9 | (1,2) | (0,2), (0,1), (0,0) | 6 |
| 11 | (4,0) | (3,0), (2,1), (1,0), (0,0) | 8 |
| 11 | (4,1) | (3,0), (2,1), (1,0), (0,0) | 8 |
| 11 | (0,3) | (0,2), (0,1), (0,0) | 6 |
| 11 | (0,1) | (1,0), (0,0) | 4 |
| 11 | (2,3) | (2,2), (2,1), (1,0), (0,0) | 8 |
| 15 | (3,1) | (2,2), (2,1), (1,0), (0,0) | 8 |

| A-Star | Node |
| ------ | ---- |
| 6 | (0,0) |
| 6 | (1,0) |
| 6 | (2,2) |
| 7 | (2,1) |
| 7 | (0,2) |
| 8 | (1,1) |
| 8 | (1,0) |
| 9 | (2,0) |
| 9 | (3,2) |
| 9 | (0,1) |
| 9 | (3,0) |
| 9 | (1,2) |
| 11 | (0,1) |
| 11 | (2,3) |
| 11 | (4,0) |
| 11 | (4,1) |
| 15 | (3,1) |

And again:

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 9 | (1,2) | (0,2), (0,1), (0,0) | 6 |
| 11 | (4,0) | (3,0), (2,1), (1,0), (0,0) | 8 |
| 11 | (4,1) | (3,0), (2,1), (1,0), (0,0) | 8 |
| 11 | (0,3) | (0,2), (0,1), (0,0) | 6 |
| 11 | (0,1) | (1,0), (0,0) | 4 |
| 11 | (2,3) | (2,2), (2,1), (1,0), (0,0) | 8 |
| 12 | (4,2) | (3,2), (2,2), (2,1), (1,0), (0,0) | 10 |
| 13 | (3,3) | (3,2), (2,2), (2,1), (1,0), (0,0) | 10 |
| 13 | (4,3) | (3,2), (2,2), (2,1), (1,0), (0,0) | 10 |
| 15 | (3,1) | (2,2), (2,1), (1,0), (0,0) | 8 |

| A-Star | Node |
| ------ | ---- |
| 6 | (0,0) |
| 6 | (1,0) |
| 6 | (2,2) |
| 7 | (2,1) |
| 7 | (0,2) |
| 8 | (1,1) |
| 8 | (1,0) |
| 9 | (2,0) |
| 9 | (3,2) |
| 9 | (0,1) |
| 9 | (3,0) |
| 9 | (1,2) |
| 11 | (0,1) |
| 11 | (2,3) |
| 11 | (4,0) |
| 11 | (4,1) |
| 12 | (4,2) |
| 13 | (4,3) |
| 13 | (3,3) |
| 15 | (3,1) |

The 9th row in our queue now is actually the endpoint `(4,3)` but as there are still unexplored routes with better efficiency (A-Star) we must carry on discovering nodes to see if there's still a better path:

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 11 | (1,3) | (1,2), (0,2), (0,1), (0,0) | 8 |
| 11 | (4,0) | (3,0), (2,1), (1,0), (0,0) | 8 |
| 11 | (4,1) | (3,0), (2,1), (1,0), (0,0) | 8 |
| 11 | (0,3) | (0,2), (0,1), (0,0) | 6 |
| 11 | (0,1) | (1,0), (0,0) | 4 |
| 11 | (2,3) | (2,2), (2,1), (1,0), (0,0) | 8 |
| 12 | (4,2) | (3,2), (2,2), (2,1), (1,0), (0,0) | 10 |
| 13 | (3,3) | (3,2), (2,2), (2,1), (1,0), (0,0) | 10 |
| 13 | (4,3) | (3,2), (2,2), (2,1), (1,0), (0,0) | 10 |
| 15 | (3,1) | (2,2), (2,1), (1,0), (0,0) | 8 |

| A-Star | Node |
| ------ | ---- |
| 6 | (0,0) |
| 6 | (1,0) |
| 6 | (2,2) |
| 7 | (2,1) |
| 7 | (0,2) |
| 8 | (1,1) |
| 8 | (1,0) |
| 9 | (2,0) |
| 9 | (3,2) |
| 9 | (0,1) |
| 9 | (3,0) |
| 9 | (1,2) |
| 11 | (1,3) |
| 11 | (0,1) |
| 11 | (2,3) |
| 11 | (4,0) |
| 11 | (4,1) |
| 12 | (4,2) |
| 13 | (4,3) |
| 13 | (3,3) |
| 15 | (3,1) |

Again (this wipes out loads of inefficient paths):

| A-Star score | Current node | Previous nodes traversed | Total Complexity |
| ------------ | ------------ | ------------------------ | -------------- |
| 13 | (4,3) | (3,2), (2,2), (2,1), (1,0), (0,0) | 10 |
| 15 | (3,1) | (2,2), (2,1), (1,0), (0,0) | 8 |

| A-Star | Node |
| ------ | ---- |
| 6 | (0,0) |
| 6 | (1,0) |
| 6 | (2,2) |
| 7 | (2,1) |
| 7 | (0,2) |
| 8 | (1,1) |
| 8 | (1,0) |
| 9 | (2,0) |
| 9 | (3,2) |
| 9 | (0,1) |
| 9 | (3,0) |
| 9 | (1,2) |
| 11 | (1,3) |
| 11 | (0,1) |
| 11 | (2,3) |
| 11 | (4,0) |
| 11 | (4,1) |
| 12 | (4,2) |
| 13 | (4,3) |
| 13 | (3,3) |
| 15 | (3,1) |

And finally we have arrived at the most optimal route from `S` to `E` as the end node has risne to the top of the queue, the path to follow is:

* (0,0) -> (1,0) -> (2,1) -> (2,2) -> (3,2) -> (4,3)

Which looks like:

```txt
                                        _______
                                       /   E   \
                               _______/  (4,3)  \
                              /       \   W:3   /
                      _______/  (3,2)  \_______/
                     /       \   W:1   /
                    /  (2,2)  \_______/
                    \   W:3   /
                     \_______/
                     /       \
             _______/  (2,1)  \
            /       \   W:3   /
    _______/  (1,0)  \_______/
   /   S   \   W:4   /
  /  (0,0)  \_______/
  \   W:6   /
   \_______/
```

## Example - Varying Complexity: A-Star for Flat Topped Odd Column Shifted Up <a name="example2"></a>

We wish to move from node `S - (0,0)` to node `E - (3,3)`. Each node has a weight `W` which is a linear representation of distance from the end point. Each hexagon contains a complexity value `C`.

```txt
                   _________               _________
                  /         \             /         \
                 /           \           /     E     \
       _________/    (1,3)    \_________/    (3,3)    \
      /         \     W:1     /         \     W:0     /
     /           \    C:2    /           \    C:2    /
    /    (0,3)    \_________/    (2,3)    \_________/
    \     W:3     /         \     W:1     /         \
     \    C:3    /           \    C:9    /           \
      \_________/    (1,2)    \_________/    (3,2)    \
      /         \     W:2     /         \     W:1     /
     /           \    C:4    /           \    C:5    /
    /    (0,2)    \_________/    (2,2)    \_________/
    \     W:3     /         \     W:2     /         \
     \    C:1    /           \    C:8    /           \
      \_________/    (1,1)    \_________/    (3,1)    \
      /         \     W:3     /         \     W:2     /
     /           \    C:9    /           \    C:4    /
    /    (0,1)    \_________/    (2,1)    \_________/
    \     W:4     /         \     W:3     /         \
     \    C:1    /           \    C:6    /           \
      \_________/    (1,0)    \_________/    (3,0)    \
      /         \     W:4     /         \     W:3     /
     /     S     \    C:2    /           \    C:3    /
    /    (0,0)    \_________/    (2,0)    \_________/
    \     W:5     /         \     W:4     /
     \    C:1    /           \    C:2    /
      \_________/             \_________/
```

In this scenarios we calculate A-Star by taking:

* `(0.5 * current_node_complexity) + (0.5 * target_node_complexity) + previous_complexities + weight`

Where `previous_complexities` is simply the previous `(0.5 * current_node_complexity) + (0.5 * target_node_complexity)`.

Our initial data set of A-Star scores:

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,0) |

Expanding from the starting node:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 5.0 | (0,1) | (0,0) | 1.0 |
| 5.5 | (1,0) | (0,0) | 1.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,0) |
| 5.0 | (0,1) |
| 5.5 | (1,0) |

Second expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 5.0 | (0,2) | (0,1), (0,0) | 2.0 |
| 5.5 | (1,0) | (0,0) | 1.5 |
| 9.0 | (1,1) | (0,1), (0,0) | 6.0 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 9.0 | (1,1) |

Third expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 5.5 | (1,0) | (0,0) | 1.5 |
| 6.5 | (1,2) | (0,2), (0,1), (0,0) | 4.5 |
| 7.0 | (0,3) | (0,2), (0,1), (0,0) | 4.0 |
| 9.0 | (1,1) | (0,1), (0,0) | 6.0 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 9.0 | (1,1) |

Forth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 6.5 | (1,2) | (0,2), (0,1), (0,0) | 4.5 |
| 7.0 | (0,3) | (0,2), (0,1), (0,0) | 4.0 |
| 7.5 | (2,0) | (1,0), (0,0) | 3.5 |
| 8.5 | (2,1) | (1,0), (0,0) | 5.5 |
| 9.0 | (1,1) | (0,1), (0,0) | 6.0 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |

Fifth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 7.0 | (0,3) | (0,2), (0,1), (0,0) | 4.0 |
| 7.5 | (2,0) | (1,0), (0,0) | 3.5 |
| 8.5 | (2,1) | (1,0), (0,0) | 5.5 |
| 9.0 | (1,1) | (0,1), (0,0) | 6.0 |
| 9.5 | (1,3) | (1,2), (0,2), (0,1), (0,0) | 8.5 |
| 12.0 | (2,3) | (1,2), (0,2), (0,1), (0,0) | 11.0 |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.5 | (1,3) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |

Sixth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 7.5 | (1,3) | (0,3), (0,2), (0,1), (0,0) | 6.5 |
| 7.5 | (2,0) | (1,0), (0,0) | 3.5 |
| 8.5 | (2,1) | (1,0), (0,0) | 5.5 |
| 9.0 | (1,1) | (0,1), (0,0) | 6.0 |
| 9.5 | (1,3) | (1,2), (0,2), (0,1), (0,0) | 8.5 |
| 12.0 | (2,3) | (1,2), (0,2), (0,1), (0,0) | 11.0 |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |

Seventh expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 7.5 | (2,0) | (1,0), (0,0) | 3.5 |
| 8.5 | (2,1) | (1,0), (0,0) | 5.5 |
| 9.0 | (1,1) | (0,1), (0,0) | 6.0 |
| 9.5 | (1,3) | (1,2), (0,2), (0,1), (0,0) | 8.5 |
| 12.0 | (2,3) | (1,2), (0,2), (0,1), (0,0) | 11.0 |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |

Eighth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 8.5 | (2,1) | (1,0), (0,0) | 5.5 |
| 9.0 | (3,0) | (2,0), (1,0), (0,0) | 6.0 |
| 9.0 | (1,1) | (0,1), (0,0) | 6.0 |
| 9.5 | (1,3) | (1,2), (0,2), (0,1), (0,0) | 8.5 |
| 12.0 | (2,3) | (1,2), (0,2), (0,1), (0,0) | 11.0 |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.0 | (3,0) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |

Ninth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 9.0 | (3,0) | (2,0), (1,0), (0,0) | 6.0 |
| 9.0 | (1,1) | (0,1), (0,0) | 6.0 |
| 9.5 | (1,3) | (1,2), (0,2), (0,1), (0,0) | 8.5 |
| 12.0 | (2,3) | (1,2), (0,2), (0,1), (0,0) | 11.0 |
| 12.5 | (3,1) | (2,1), (1,0), (0,0) | 10.5 |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.0 | (3,0) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |
| 12.5 | (3,1) |

Tenth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 9.0 | (1,1) | (0,1), (0,0) | 6.0 |
| 9.5 | (1,3) | (1,2), (0,2), (0,1), (0,0) | 8.5 |
| 11.5 | (3,1) | (3,0), (2,0), (1,0), (0,0) | 9.5 |
| 12.0 | (2,3) | (1,2), (0,2), (0,1), (0,0) | 11.0 |
| 12.5 | (3,1) | (2,1), (1,0), (0,0) | 10.5 |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.0 | (3,0) |
| 11.5 | (3,1) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |

Eleventh expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 9.5 | (1,3) | (1,2), (0,2), (0,1), (0,0) | 8.5 |
| 11.5 | (3,1) | (3,0), (2,0), (1,0), (0,0) | 9.5 |
| 12.0 | (2,3) | (1,2), (0,2), (0,1), (0,0) | 11.0 |
| 12.5 | (3,1) | (2,1), (1,0), (0,0) | 10.5 |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.0 | (3,0) |
| 11.5 | (3,1) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |

Twelveth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 11.5 | (3,1) | (3,0), (2,0), (1,0), (0,0) | 9.5 |
| 12.0 | (2,3) | (1,2), (0,2), (0,1), (0,0) | 11.0 |
| 12.5 | (3,1) | (2,1), (1,0), (0,0) | 10.5 |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.0 | (3,0) |
| 11.5 | (3,1) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |

Thirteenth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 12.0 | (2,3) | (1,2), (0,2), (0,1), (0,0) | 11.0 |
| 12.5 | (3,1) | (2,1), (1,0), (0,0) | 10.5 |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |
| 16.0 | (3,2) | (3,1), (3,0), (2,0), (1,0), (0,0) | 15.0 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.0 | (3,0) |
| 11.5 | (3,1) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |
| 16.0 | (3,2) |

Fourteenth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 12.5 | (3,1) | (2,1), (1,0), (0,0) | 10.5 |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |
| 16.0 | (3,2) | (3,1), (3,0), (2,0), (1,0), (0,0) | 15.0 |
| 16.5 | (3,3) | (2,3), (1,2), (0,2), (0,1), (0,0) | 16.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.0 | (3,0) |
| 11.5 | (3,1) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |
| 16.0 | (3,2) |
| 16.5 | (3,3) |

Fifthteenth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 12.5 | (2,2) | (1,2), (0,2), (0,1), (0,0) | 10.5 |
| 16.0 | (3,2) | (3,1), (3,0), (2,0), (1,0), (0,0) | 15.0 |
| 16.5 | (3,3) | (2,3), (1,2), (0,2), (0,1), (0,0) | 16.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.0 | (3,0) |
| 11.5 | (3,1) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |
| 16.0 | (3,2) |
| 16.5 | (3,3) |

Sixteenth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 16.0 | (3,2) | (3,1), (3,0), (2,0), (1,0), (0,0) | 15.0 |
| 16.5 | (3,3) | (2,3), (1,2), (0,2), (0,1), (0,0) | 16.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.0 | (3,0) |
| 11.5 | (3,1) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |
| 16.0 | (3,2) |
| 16.5 | (3,3) |

Seventeenth expansion:

| A-Star score | Current node | Previous nodes traversed | Total Complexities |
| ------------ | ------------ | ------------------------ | -------------- |
| 16.5 | (3,3) | (2,3), (1,2), (0,2), (0,1), (0,0) | 16.5 |

| A-Star | Node |
| ------ | ---- |
| 5.0 | (0,1) |
| 5.0 | (0,2) |
| 5.5 | (0,0) |
| 5.5 | (1,0) |
| 6.5 | (1,2) |
| 7.0 | (0,3) |
| 7.5 | (2,0) |
| 7.5 | (1,3) |
| 8.5 | (2,1) |
| 9.0 | (1,1) |
| 9.0 | (3,0) |
| 11.5 | (3,1) |
| 12.0 | (2,3) |
| 12.5 | (2,2) |
| 16.0 | (3,2) |
| 16.5 | (3,3) |

The end node has risen to the top of the queue so we now know the most optimal route, which to follow:

* (0,0) -> (0,1) -> (0,2) -> (1,2) -> (2,3) -> (3,3)

Which looks like:

```txt
                                           _________
                                          /         \
                                         /     E     \
                               _________/    (3,3)    \
                              /         \     W:0     /
                             /           \    C:2    /
                   _________/    (2,3)    \_________/
                  /         \     W:1     /
                 /           \    C:9    /
       _________/    (1,2)    \_________/
      /         \     W:2     /
     /           \    C:4    /
    /    (0,2)    \_________/
    \     W:3     /
     \    C:1    /
      \_________/
      /         \
     /           \
    /    (0,1)    \
    \     W:4     /
     \    C:1    /
      \_________/
      /         \
     /     S     \
    /    (0,0)    \
    \     W:5     /
     \    C:1    /
      \_________/
```
