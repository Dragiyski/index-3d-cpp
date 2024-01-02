Unmix the problems.

There are 2 problems to be solved here:

1. Generating an index;
2. Storing the data and the index;

# Data Storage

The original idea is to store 3-4 separate buffers.

1. `int_data` a variable array of unsigned integers per object;
2. `float_data` a variable array of floating-point numbers per object;
3. `object_data` a fixed size array of unsigned integer per object, usually 4;

To address repetitions of float/int elements we introduce more buffers:

1. `float_data` an array of all floating-point values used, sorted.
2. `double_list_data` an array of `uint[2]` for all 2-elements vectors.
3. `triple_list_data` an array of `uint[3]` for all 3-elements vectors.
4. `index_data` an variable array of `uint` per object.
5. `object_data` a fixed `uint[8]` per object.

Every node will have exactly one entry in the object data. It should contain:

* `ID:uint` - the node type;
* `index_base:uint` - the index within the `index_data` containing the object information;
* `index_length:uint` - the amount of data within the `index_data` for that object;
* `parent_index:uint` - the index within the `object_data` for the parent of this node;
* `child_base_index` - the index within the `index_data` where the children indices are contained; It can be ignored or reused for other purposes on certain `ID`s and/or when `child_count` is `0`.
* `child_count` - the number of children in the `index_data`
* `prev_sibling` - the index within the `object_data` to the previous sibling.
* `next_sibling` - the index within the `object_data` to the next sibling.

Object at index 0 should be a NULL object with ID=0. It does not represents any data and other values must be ignored. References to `object_data` that are 0 are thus considered `null` references, i.e. `parent_node` = 0 should be ignored and considered a root object. Some information may be stored in the NULL object like:

* Depth of tree
* Total number of nodes
* Index of the root node for raytracing