import random

def insert(heap, value_map, value, item):
    if value in heap:
        value_map[value].append(item)
    else:
        value_map[value] = [item]
        
        heap.append(None)
        index = len(heap) - 1
        

        while index > 0 and value < heap[(index - 1) //2]:
            heap[index] = heap[(index-1)//2]
            index = (index-1)//2

        heap[index] = value

def minify(heap, index):
    size = len(heap)
    left_child = 2 * index + 1
    right_child = 2 * index + 2
    smallest = index
    
    if left_child < size and heap[left_child] < heap[smallest]:
        smallest = left_child
    
    if right_child < size and heap[right_child ] < heap[smallest]:
        smallest = right_child

    if smallest != index : 
        heap[smallest], heap[index] = heap[index], heap[smallest]
        minify(heap, smallest)

def pop(heap, value_map):
    smallest_value = heap[0]

    selected_item = value_map[smallest_value].pop(random.randrange(len(value_map[smallest_value])))

    if len(value_map[smallest_value]) == 0:
        del value_map[smallest_value]

        heap[0] = heap[-1]
        heap.pop()

        if heap:
            minify(heap, 0)

    return selected_item 