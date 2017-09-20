package box2d

import (
	"math"
)

type B2TreeQueryCallback func(nodeId int) bool

type B2TreeRayCastCallback func(input B2RayCastInput, nodeId int) float64

const B2_nullNode = -1

type B2TreeNode struct {

	/// Enlarged AABB
	Aabb B2AABB

	UserData interface{}

	// union
	// {
	Parent int
	Next   int
	//};

	Child1 int
	Child2 int

	// leaf = 0, free node = -1
	Height int
}

func (node B2TreeNode) IsLeaf() bool {
	return node.Child1 == B2_nullNode
}

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
type B2DynamicTree struct {

	// Public members:
	// None

	// Private members:
	M_root int

	M_nodes        []B2TreeNode
	M_nodeCount    int
	M_nodeCapacity int

	M_freeList int

	/// This is used to incrementally traverse the tree for re-balancing.
	M_path int

	M_insertionCount int
}

func (tree B2DynamicTree) GetUserData(proxyId int) interface{} {
	B2Assert(0 <= proxyId && proxyId < tree.M_nodeCapacity)
	return tree.M_nodes[proxyId].UserData
}

func (tree B2DynamicTree) GetFatAABB(proxyId int) B2AABB {
	B2Assert(0 <= proxyId && proxyId < tree.M_nodeCapacity)
	return tree.M_nodes[proxyId].Aabb
}

func (tree *B2DynamicTree) Query(queryCallback B2TreeQueryCallback, aabb B2AABB) {
	stack := NewB2GrowableStack()
	stack.Push(tree.M_root)

	for stack.GetCount() > 0 {
		nodeId := stack.Pop().(int)
		if nodeId == B2_nullNode {
			continue
		}

		node := &tree.M_nodes[nodeId]

		if B2TestOverlapBoundingBoxes(node.Aabb, aabb) {
			if node.IsLeaf() {
				proceed := queryCallback(nodeId)
				if proceed == false {
					return
				}
			} else {
				stack.Push(node.Child1)
				stack.Push(node.Child2)
			}
		}
	}
}

func (tree B2DynamicTree) RayCast(rayCastCallback B2TreeRayCastCallback, input B2RayCastInput) {

	p1 := input.P1
	p2 := input.P2
	r := B2Vec2Sub(p2, p1)
	B2Assert(r.LengthSquared() > 0.0)
	r.Normalize()

	// v is perpendicular to the segment.
	v := B2Vec2CrossScalarVector(1.0, r)
	abs_v := B2Vec2Abs(v)

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	maxFraction := input.MaxFraction

	// Build a bounding box for the segment.
	segmentAABB := MakeB2AABB()
	{
		t := B2Vec2Add(p1, B2Vec2MulScalar(maxFraction, B2Vec2Sub(p2, p1)))
		segmentAABB.LowerBound = B2Vec2Min(p1, t)
		segmentAABB.UpperBound = B2Vec2Max(p1, t)
	}

	stack := NewB2GrowableStack()
	stack.Push(tree.M_root)

	for stack.GetCount() > 0 {
		nodeId := stack.Pop().(int)
		if nodeId == B2_nullNode {
			continue
		}

		node := &tree.M_nodes[nodeId]

		if B2TestOverlapBoundingBoxes(node.Aabb, segmentAABB) == false {
			continue
		}

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		c := node.Aabb.GetCenter()
		h := node.Aabb.GetExtents()

		separation := math.Abs(B2Vec2Dot(v, B2Vec2Sub(p1, c))) - B2Vec2Dot(abs_v, h)
		if separation > 0.0 {
			continue
		}

		if node.IsLeaf() {
			subInput := MakeB2RayCastInput()
			subInput.P1 = input.P1
			subInput.P2 = input.P2
			subInput.MaxFraction = maxFraction

			value := rayCastCallback(subInput, nodeId)

			if value == 0.0 {
				// The client has terminated the ray cast.
				return
			}

			if value > 0.0 {
				// Update segment bounding box.
				maxFraction = value
				t := B2Vec2Add(p1, B2Vec2MulScalar(maxFraction, B2Vec2Sub(p2, p1)))
				segmentAABB.LowerBound = B2Vec2Min(p1, t)
				segmentAABB.UpperBound = B2Vec2Max(p1, t)
			}
		} else {
			stack.Push(node.Child1)
			stack.Push(node.Child2)
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// B2DynamicTree.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func MakeB2DynamicTree() B2DynamicTree {

	tree := B2DynamicTree{}
	tree.M_root = B2_nullNode

	tree.M_nodeCapacity = 16
	tree.M_nodeCount = 0
	tree.M_nodes = make([]B2TreeNode, tree.M_nodeCapacity)

	// Build a linked list for the free list.
	for i := 0; i < tree.M_nodeCapacity-1; i++ {
		tree.M_nodes[i].Next = i + 1
		tree.M_nodes[i].Height = -1
	}

	tree.M_nodes[tree.M_nodeCapacity-1].Next = B2_nullNode
	tree.M_nodes[tree.M_nodeCapacity-1].Height = -1
	tree.M_freeList = 0

	tree.M_path = 0

	tree.M_insertionCount = 0

	return tree
}

// func (tree *B2DynamicTree) ~b2DynamicTree() {
// 	// This frees the entire tree in one shot.
// 	b2Free(tree.M_nodes);
// }

// Allocate a node from the pool. Grow the pool if necessary.
func (tree *B2DynamicTree) AllocateNode() int {

	// Expand the node pool as needed.
	if tree.M_freeList == B2_nullNode {
		B2Assert(tree.M_nodeCount == tree.M_nodeCapacity)

		// The free list is empty. Rebuild a bigger pool.
		tree.M_nodes = append(tree.M_nodes, make([]B2TreeNode, tree.M_nodeCapacity)...)
		tree.M_nodeCapacity *= 2

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
		for i := tree.M_nodeCount; i < tree.M_nodeCapacity-1; i++ {
			tree.M_nodes[i].Next = i + 1
			tree.M_nodes[i].Height = -1
		}

		tree.M_nodes[tree.M_nodeCapacity-1].Next = B2_nullNode
		tree.M_nodes[tree.M_nodeCapacity-1].Height = -1
		tree.M_freeList = tree.M_nodeCount
	}

	// Peel a node off the free list.
	nodeId := tree.M_freeList
	tree.M_freeList = tree.M_nodes[nodeId].Next
	tree.M_nodes[nodeId].Parent = B2_nullNode
	tree.M_nodes[nodeId].Child1 = B2_nullNode
	tree.M_nodes[nodeId].Child2 = B2_nullNode
	tree.M_nodes[nodeId].Height = 0
	tree.M_nodes[nodeId].UserData = nil
	tree.M_nodeCount++

	return nodeId
}

// Return a node to the pool.
func (tree *B2DynamicTree) FreeNode(nodeId int) {
	B2Assert(0 <= nodeId && nodeId < tree.M_nodeCapacity)
	B2Assert(0 < tree.M_nodeCount)
	tree.M_nodes[nodeId].Next = tree.M_freeList
	tree.M_nodes[nodeId].Height = -1
	tree.M_freeList = nodeId
	tree.M_nodeCount--
}

// Create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool.
func (tree *B2DynamicTree) CreateProxy(aabb B2AABB, userData interface{}) int {

	proxyId := tree.AllocateNode()

	// Fatten the aabb.
	r := MakeB2Vec2(B2_aabbExtension, B2_aabbExtension)
	tree.M_nodes[proxyId].Aabb.LowerBound = B2Vec2Sub(aabb.LowerBound, r)
	tree.M_nodes[proxyId].Aabb.UpperBound = B2Vec2Add(aabb.UpperBound, r)
	tree.M_nodes[proxyId].UserData = userData
	tree.M_nodes[proxyId].Height = 0

	tree.InsertLeaf(proxyId)

	return proxyId
}

func (tree *B2DynamicTree) DestroyProxy(proxyId int) {
	B2Assert(0 <= proxyId && proxyId < tree.M_nodeCapacity)
	B2Assert(tree.M_nodes[proxyId].IsLeaf())

	tree.RemoveLeaf(proxyId)
	tree.FreeNode(proxyId)
}

func (tree *B2DynamicTree) MoveProxy(proxyId int, aabb B2AABB, displacement B2Vec2) bool {

	B2Assert(0 <= proxyId && proxyId < tree.M_nodeCapacity)

	B2Assert(tree.M_nodes[proxyId].IsLeaf())

	if tree.M_nodes[proxyId].Aabb.Contains(aabb) {
		return false
	}

	tree.RemoveLeaf(proxyId)

	// Extend AABB.
	b := aabb.Clone()
	r := MakeB2Vec2(B2_aabbExtension, B2_aabbExtension)
	b.LowerBound = B2Vec2Sub(b.LowerBound, r)
	b.UpperBound = B2Vec2Add(b.UpperBound, r)

	// Predict AABB displacement.
	d := B2Vec2MulScalar(B2_aabbMultiplier, displacement)

	if d.X < 0.0 {
		b.LowerBound.X += d.X
	} else {
		b.UpperBound.X += d.X
	}

	if d.Y < 0.0 {
		b.LowerBound.Y += d.Y
	} else {
		b.UpperBound.Y += d.Y
	}

	tree.M_nodes[proxyId].Aabb = b

	tree.InsertLeaf(proxyId)

	return true
}

func (tree *B2DynamicTree) InsertLeaf(leaf int) {
	tree.M_insertionCount++

	if tree.M_root == B2_nullNode {
		tree.M_root = leaf
		tree.M_nodes[tree.M_root].Parent = B2_nullNode
		return
	}

	// Find the best sibling for this node
	leafAABB := tree.M_nodes[leaf].Aabb
	index := tree.M_root
	for tree.M_nodes[index].IsLeaf() == false {
		child1 := tree.M_nodes[index].Child1
		child2 := tree.M_nodes[index].Child2

		area := tree.M_nodes[index].Aabb.GetPerimeter()

		combinedAABB := NewB2AABB()
		combinedAABB.CombineTwoInPlace(tree.M_nodes[index].Aabb, leafAABB)
		combinedArea := combinedAABB.GetPerimeter()

		// Cost of creating a new parent for this node and the new leaf
		cost := 2.0 * combinedArea

		// Minimum cost of pushing the leaf further down the tree
		inheritanceCost := 2.0 * (combinedArea - area)

		// Cost of descending into child1
		cost1 := 0.0
		if tree.M_nodes[child1].IsLeaf() {
			aabb := NewB2AABB()
			aabb.CombineTwoInPlace(leafAABB, tree.M_nodes[child1].Aabb)
			cost1 = aabb.GetPerimeter() + inheritanceCost
		} else {
			aabb := NewB2AABB()
			aabb.CombineTwoInPlace(leafAABB, tree.M_nodes[child1].Aabb)
			oldArea := tree.M_nodes[child1].Aabb.GetPerimeter()
			newArea := aabb.GetPerimeter()
			cost1 = (newArea - oldArea) + inheritanceCost
		}

		// Cost of descending into child2
		cost2 := 0.0
		if tree.M_nodes[child2].IsLeaf() {
			aabb := NewB2AABB()
			aabb.CombineTwoInPlace(leafAABB, tree.M_nodes[child2].Aabb)
			cost2 = aabb.GetPerimeter() + inheritanceCost
		} else {
			aabb := NewB2AABB()
			aabb.CombineTwoInPlace(leafAABB, tree.M_nodes[child2].Aabb)
			oldArea := tree.M_nodes[child2].Aabb.GetPerimeter()
			newArea := aabb.GetPerimeter()
			cost2 = newArea - oldArea + inheritanceCost
		}

		// Descend according to the minimum cost.
		if cost < cost1 && cost < cost2 {
			break
		}

		// Descend
		if cost1 < cost2 {
			index = child1
		} else {
			index = child2
		}
	}

	sibling := index

	// Create a new parent.
	oldParent := tree.M_nodes[sibling].Parent
	newParent := tree.AllocateNode()
	tree.M_nodes[newParent].Parent = oldParent
	tree.M_nodes[newParent].UserData = nil
	tree.M_nodes[newParent].Aabb.CombineTwoInPlace(leafAABB, tree.M_nodes[sibling].Aabb)
	tree.M_nodes[newParent].Height = tree.M_nodes[sibling].Height + 1

	if oldParent != B2_nullNode {
		// The sibling was not the root.
		if tree.M_nodes[oldParent].Child1 == sibling {
			tree.M_nodes[oldParent].Child1 = newParent
		} else {
			tree.M_nodes[oldParent].Child2 = newParent
		}

		tree.M_nodes[newParent].Child1 = sibling
		tree.M_nodes[newParent].Child2 = leaf
		tree.M_nodes[sibling].Parent = newParent
		tree.M_nodes[leaf].Parent = newParent
	} else {
		// The sibling was the root.
		tree.M_nodes[newParent].Child1 = sibling
		tree.M_nodes[newParent].Child2 = leaf
		tree.M_nodes[sibling].Parent = newParent
		tree.M_nodes[leaf].Parent = newParent
		tree.M_root = newParent
	}

	// Walk back up the tree fixing heights and AABBs
	index = tree.M_nodes[leaf].Parent
	for index != B2_nullNode {
		index = tree.Balance(index)

		child1 := tree.M_nodes[index].Child1
		child2 := tree.M_nodes[index].Child2

		B2Assert(child1 != B2_nullNode)
		B2Assert(child2 != B2_nullNode)

		tree.M_nodes[index].Height = 1 + MaxInt(tree.M_nodes[child1].Height, tree.M_nodes[child2].Height)
		tree.M_nodes[index].Aabb.CombineTwoInPlace(tree.M_nodes[child1].Aabb, tree.M_nodes[child2].Aabb)

		index = tree.M_nodes[index].Parent
	}

	//Validate();
}

func (tree *B2DynamicTree) RemoveLeaf(leaf int) {
	if leaf == tree.M_root {
		tree.M_root = B2_nullNode
		return
	}

	parent := tree.M_nodes[leaf].Parent
	grandParent := tree.M_nodes[parent].Parent
	sibling := 0
	if tree.M_nodes[parent].Child1 == leaf {
		sibling = tree.M_nodes[parent].Child2
	} else {
		sibling = tree.M_nodes[parent].Child1
	}

	if grandParent != B2_nullNode {
		// Destroy parent and connect sibling to grandParent.
		if tree.M_nodes[grandParent].Child1 == parent {
			tree.M_nodes[grandParent].Child1 = sibling
		} else {
			tree.M_nodes[grandParent].Child2 = sibling
		}
		tree.M_nodes[sibling].Parent = grandParent
		tree.FreeNode(parent)

		// Adjust ancestor bounds.
		index := grandParent
		for index != B2_nullNode {
			index = tree.Balance(index)

			child1 := tree.M_nodes[index].Child1
			child2 := tree.M_nodes[index].Child2

			tree.M_nodes[index].Aabb.CombineTwoInPlace(tree.M_nodes[child1].Aabb, tree.M_nodes[child2].Aabb)
			tree.M_nodes[index].Height = 1 + MaxInt(tree.M_nodes[child1].Height, tree.M_nodes[child2].Height)

			index = tree.M_nodes[index].Parent
		}
	} else {
		tree.M_root = sibling
		tree.M_nodes[sibling].Parent = B2_nullNode
		tree.FreeNode(parent)
	}

	// //Validate();
}

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
func (tree *B2DynamicTree) Balance(iA int) int {
	B2Assert(iA != B2_nullNode)

	A := &tree.M_nodes[iA]
	if A.IsLeaf() || A.Height < 2 {
		return iA
	}

	iB := A.Child1
	iC := A.Child2
	B2Assert(0 <= iB && iB < tree.M_nodeCapacity)
	B2Assert(0 <= iC && iC < tree.M_nodeCapacity)

	B := &tree.M_nodes[iB]
	C := &tree.M_nodes[iC]

	balance := C.Height - B.Height

	// Rotate C up
	if balance > 1 {
		iF := C.Child1
		iG := C.Child2
		B2Assert(0 <= iF && iF < tree.M_nodeCapacity)
		B2Assert(0 <= iG && iG < tree.M_nodeCapacity)
		F := &tree.M_nodes[iF]
		G := &tree.M_nodes[iG]

		// Swap A and C
		C.Child1 = iA
		C.Parent = A.Parent
		A.Parent = iC

		// A's old parent should point to C
		if C.Parent != B2_nullNode {
			if tree.M_nodes[C.Parent].Child1 == iA {
				tree.M_nodes[C.Parent].Child1 = iC
			} else {
				B2Assert(tree.M_nodes[C.Parent].Child2 == iA)
				tree.M_nodes[C.Parent].Child2 = iC
			}
		} else {
			tree.M_root = iC
		}

		// Rotate
		if F.Height > G.Height {
			C.Child2 = iF
			A.Child2 = iG
			G.Parent = iA
			A.Aabb.CombineTwoInPlace(B.Aabb, G.Aabb)
			C.Aabb.CombineTwoInPlace(A.Aabb, F.Aabb)

			A.Height = 1 + MaxInt(B.Height, G.Height)
			C.Height = 1 + MaxInt(A.Height, F.Height)
		} else {
			C.Child2 = iG
			A.Child2 = iF
			F.Parent = iA
			A.Aabb.CombineTwoInPlace(B.Aabb, F.Aabb)
			C.Aabb.CombineTwoInPlace(A.Aabb, G.Aabb)

			A.Height = 1 + MaxInt(B.Height, F.Height)
			C.Height = 1 + MaxInt(A.Height, G.Height)
		}

		return iC
	}

	// Rotate B up
	if balance < -1 {
		iD := B.Child1
		iE := B.Child2
		B2Assert(0 <= iD && iD < tree.M_nodeCapacity)
		B2Assert(0 <= iE && iE < tree.M_nodeCapacity)

		D := &tree.M_nodes[iD]
		E := &tree.M_nodes[iE]

		// Swap A and B
		B.Child1 = iA
		B.Parent = A.Parent
		A.Parent = iB

		// A's old parent should point to B
		if B.Parent != B2_nullNode {
			if tree.M_nodes[B.Parent].Child1 == iA {
				tree.M_nodes[B.Parent].Child1 = iB
			} else {
				B2Assert(tree.M_nodes[B.Parent].Child2 == iA)
				tree.M_nodes[B.Parent].Child2 = iB
			}
		} else {
			tree.M_root = iB
		}

		// Rotate
		if D.Height > E.Height {
			B.Child2 = iD
			A.Child1 = iE
			E.Parent = iA
			A.Aabb.CombineTwoInPlace(C.Aabb, E.Aabb)
			B.Aabb.CombineTwoInPlace(A.Aabb, D.Aabb)

			A.Height = 1 + MaxInt(C.Height, E.Height)
			B.Height = 1 + MaxInt(A.Height, D.Height)
		} else {
			B.Child2 = iE
			A.Child1 = iD
			D.Parent = iA
			A.Aabb.CombineTwoInPlace(C.Aabb, D.Aabb)
			B.Aabb.CombineTwoInPlace(A.Aabb, E.Aabb)

			A.Height = 1 + MaxInt(C.Height, D.Height)
			B.Height = 1 + MaxInt(A.Height, E.Height)
		}

		return iB
	}

	return iA
}

func (tree B2DynamicTree) GetHeight() int {
	if tree.M_root == B2_nullNode {
		return 0
	}

	return tree.M_nodes[tree.M_root].Height
}

//
func (tree B2DynamicTree) GetAreaRatio() float64 {
	if tree.M_root == B2_nullNode {
		return 0.0
	}

	root := &tree.M_nodes[tree.M_root]
	rootArea := root.Aabb.GetPerimeter()

	totalArea := 0.0
	for i := 0; i < tree.M_nodeCapacity; i++ {
		node := &tree.M_nodes[i]
		if node.Height < 0 {
			// Free node in pool
			continue
		}

		totalArea += node.Aabb.GetPerimeter()
	}

	return totalArea / rootArea
}

// Compute the height of a sub-tree.
func (tree B2DynamicTree) ComputeHeight(nodeId int) int {
	B2Assert(0 <= nodeId && nodeId < tree.M_nodeCapacity)
	node := &tree.M_nodes[nodeId]

	if node.IsLeaf() {
		return 0
	}

	height1 := tree.ComputeHeight(node.Child1)
	height2 := tree.ComputeHeight(node.Child2)
	return 1 + MaxInt(height1, height2)
}

func (tree B2DynamicTree) ComputeTotalHeight() int {
	return tree.ComputeHeight(tree.M_root)
}

func (tree B2DynamicTree) ValidateStructure(index int) {
	if index == B2_nullNode {
		return
	}

	if index == tree.M_root {
		B2Assert(tree.M_nodes[index].Parent == B2_nullNode)
	}

	node := &tree.M_nodes[index]

	child1 := node.Child1
	child2 := node.Child2

	if node.IsLeaf() {
		B2Assert(child1 == B2_nullNode)
		B2Assert(child2 == B2_nullNode)
		B2Assert(node.Height == 0)
		return
	}

	B2Assert(0 <= child1 && child1 < tree.M_nodeCapacity)
	B2Assert(0 <= child2 && child2 < tree.M_nodeCapacity)

	B2Assert(tree.M_nodes[child1].Parent == index)
	B2Assert(tree.M_nodes[child2].Parent == index)

	tree.ValidateStructure(child1)
	tree.ValidateStructure(child2)
}

func (tree B2DynamicTree) ValidateMetrics(index int) {
	if index == B2_nullNode {
		return
	}

	node := &tree.M_nodes[index]

	child1 := node.Child1
	child2 := node.Child2

	if node.IsLeaf() {
		B2Assert(child1 == B2_nullNode)
		B2Assert(child2 == B2_nullNode)
		B2Assert(node.Height == 0)
		return
	}

	B2Assert(0 <= child1 && child1 < tree.M_nodeCapacity)
	B2Assert(0 <= child2 && child2 < tree.M_nodeCapacity)

	height1 := tree.M_nodes[child1].Height
	height2 := tree.M_nodes[child2].Height
	height := 1 + MaxInt(height1, height2)
	B2Assert(node.Height == height)

	aabb := NewB2AABB()
	aabb.CombineTwoInPlace(tree.M_nodes[child1].Aabb, tree.M_nodes[child2].Aabb)

	B2Assert(aabb.LowerBound == node.Aabb.LowerBound)
	B2Assert(aabb.UpperBound == node.Aabb.UpperBound)

	tree.ValidateMetrics(child1)
	tree.ValidateMetrics(child2)
}

func (tree B2DynamicTree) Validate() {
}

func (tree B2DynamicTree) GetMaxBalance() int {
	maxBalance := 0
	for i := 0; i < tree.M_nodeCapacity; i++ {
		node := &tree.M_nodes[i]
		if node.Height <= 1 {
			continue
		}

		B2Assert(node.IsLeaf() == false)

		child1 := node.Child1
		child2 := node.Child2
		balance := AbsInt(tree.M_nodes[child2].Height - tree.M_nodes[child1].Height)
		maxBalance = MaxInt(maxBalance, balance)
	}

	return maxBalance
}

func (tree *B2DynamicTree) RebuildBottomUp() {
	//int* nodes = (int*)b2Alloc(m_nodeCount * sizeof(int));
	nodes := make([]int, tree.M_nodeCount)
	count := 0

	// Build array of leaves. Free the rest.
	for i := 0; i < tree.M_nodeCapacity; i++ {
		if tree.M_nodes[i].Height < 0 {
			// free node in pool
			continue
		}

		if tree.M_nodes[i].IsLeaf() {
			tree.M_nodes[i].Parent = B2_nullNode
			nodes[count] = i
			count++
		} else {
			tree.FreeNode(i)
		}
	}

	for count > 1 {
		minCost := B2_maxFloat
		iMin := -1
		jMin := -1

		for i := 0; i < count; i++ {
			aabbi := tree.M_nodes[nodes[i]].Aabb

			for j := i + 1; j < count; j++ {
				aabbj := tree.M_nodes[nodes[j]].Aabb
				b := NewB2AABB()
				b.CombineTwoInPlace(aabbi, aabbj)
				cost := b.GetPerimeter()
				if cost < minCost {
					iMin = i
					jMin = j
					minCost = cost
				}
			}
		}

		index1 := nodes[iMin]
		index2 := nodes[jMin]
		child1 := &tree.M_nodes[index1]
		child2 := &tree.M_nodes[index2]

		parentIndex := tree.AllocateNode()
		parent := &tree.M_nodes[parentIndex]
		parent.Child1 = index1
		parent.Child2 = index2
		parent.Height = 1 + MaxInt(child1.Height, child2.Height)
		parent.Aabb.CombineTwoInPlace(child1.Aabb, child2.Aabb)
		parent.Parent = B2_nullNode

		child1.Parent = parentIndex
		child2.Parent = parentIndex

		nodes[jMin] = nodes[count-1]
		nodes[iMin] = parentIndex
		count--
	}

	tree.M_root = nodes[0]
	//b2Free(nodes)

	tree.Validate()
}

func (tree *B2DynamicTree) ShiftOrigin(newOrigin B2Vec2) {
	// Build array of leaves. Free the rest.
	for i := 0; i < tree.M_nodeCapacity; i++ {
		tree.M_nodes[i].Aabb.LowerBound.OperatorMinusInplace(newOrigin)
		tree.M_nodes[i].Aabb.UpperBound.OperatorMinusInplace(newOrigin)
	}
}
