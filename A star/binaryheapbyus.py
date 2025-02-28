class BinaryHeap:
    def __init__(self):
        self.heap = []

    def is_empty(self): # empty heap
        return len(self.heap) == 0

    def push(self, item):
        """Push item into the heap while ensuring tuple-based comparison."""
        assert isinstance(item, tuple), "Heap only supports tuple-based priorities."
        self.heap.append(item)
        self.heap_up(len(self.heap) - 1)
    def empty(self):  # ✅ Alias method for compatibility
        return self.is_empty()

    def pop(self): #swap out the last item and item to be removed and then remove the last item then heap down
        if self.is_empty(): #if heap is empty then return nothing
            return None
        
        self.swap(0, len(self.heap) - 1) 

        item = self.heap.pop()

        if not self.is_empty():
            self.heap_down(0)

        return item

    def heap_up(self, index): #if current is smaller than parent then swap
        parent = (index - 1) // 2

        if index > 0 and self.heap[index] < self.heap[parent]:
            self.swap(index, parent)

            self.heap_up(parent)

    def heap_down(self, index): #find the smallest child if not current node swap
        left = 2 * index + 1
        right = 2 * index + 2
        smallest = index

        if left < len(self.heap) and self.heap[left] < self.heap[smallest]:
            smallest = left

        if right < len(self.heap) and self.heap[right] < self.heap[smallest]:
            smallest = right

        if smallest != index:
            self.swap(index, smallest)

            self.heap_down(smallest)

    def swap(self, A, B):
        temp = self.heap[A]
        self.heap[A] = self.heap[B]
        self.heap[B] = temp

