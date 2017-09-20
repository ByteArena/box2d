package box2d

// Adapted from https://gist.github.com/bemasher/1777766

type B2GrowableStack struct {
	top  *StackElement
	size int
}

func NewB2GrowableStack() *B2GrowableStack {
	return &B2GrowableStack{
		top:  nil,
		size: 0,
	}
}

type StackElement struct {
	value interface{} // All types satisfy the empty interface, so we can store anything here.
	next  *StackElement
}

// Return the stack's length
func (s B2GrowableStack) GetCount() int {
	return s.size
}

// Push a new element onto the stack
func (s *B2GrowableStack) Push(value interface{}) {
	s.top = &StackElement{value, s.top}
	s.size++
}

// Remove the top element from the stack and return it's value
// If the stack is empty, return nil
func (s *B2GrowableStack) Pop() (value interface{}) {
	if s.size > 0 {
		value, s.top = s.top.value, s.top.next
		s.size--
		return
	}
	return nil
}
