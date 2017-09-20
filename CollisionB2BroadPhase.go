package box2d

import (
	"sort"
)

type B2BroadPhaseAddPairCallback func(userDataA interface{}, userDataB interface{})

type B2Pair struct {
	ProxyIdA int
	ProxyIdB int
}

const E_nullProxy = -1

type B2BroadPhase struct {
	M_tree B2DynamicTree

	M_proxyCount int

	M_moveBuffer   []int
	M_moveCapacity int
	M_moveCount    int

	M_pairBuffer   []B2Pair
	M_pairCapacity int
	M_pairCount    int

	M_queryProxyId int
}

type PairByLessThan []B2Pair

func (a PairByLessThan) Len() int      { return len(a) }
func (a PairByLessThan) Swap(i, j int) { a[i], a[j] = a[j], a[i] }
func (a PairByLessThan) Less(i, j int) bool {
	return B2PairLessThan(a[i], a[j])
}

/// This is used to sort pairs.
func B2PairLessThan(pair1 B2Pair, pair2 B2Pair) bool {
	if pair1.ProxyIdA < pair2.ProxyIdA {
		return true
	}

	if pair1.ProxyIdA == pair2.ProxyIdA {
		return pair1.ProxyIdB < pair2.ProxyIdB
	}

	return false
}

func (bp B2BroadPhase) GetUserData(proxyId int) interface{} {
	return bp.M_tree.GetUserData(proxyId)
}

func (bp B2BroadPhase) TestOverlap(proxyIdA int, proxyIdB int) bool {
	return B2TestOverlapBoundingBoxes(
		bp.M_tree.GetFatAABB(proxyIdA),
		bp.M_tree.GetFatAABB(proxyIdB),
	)
}

func (bp B2BroadPhase) GetFatAABB(proxyId int) B2AABB {
	return bp.M_tree.GetFatAABB(proxyId)
}

func (bp B2BroadPhase) GetProxyCount() int {
	return bp.M_proxyCount
}

func (bp B2BroadPhase) GetTreeHeight() int {
	return bp.M_tree.GetHeight()
}

func (bp B2BroadPhase) GetTreeBalance() int {
	return bp.M_tree.GetMaxBalance()
}

func (bp B2BroadPhase) GetTreeQuality() float64 {
	return bp.M_tree.GetAreaRatio()
}

func (bp *B2BroadPhase) UpdatePairs(addPairCallback B2BroadPhaseAddPairCallback) {
	// Reset pair buffer
	bp.M_pairCount = 0

	// Perform tree queries for all moving proxies.
	for i := 0; i < bp.M_moveCount; i++ {
		bp.M_queryProxyId = bp.M_moveBuffer[i]
		if bp.M_queryProxyId == E_nullProxy {
			continue
		}

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a pair that may touch later.
		fatAABB := bp.M_tree.GetFatAABB(bp.M_queryProxyId)

		// Query tree, create pairs and add them pair buffer.
		bp.M_tree.Query(bp.QueryCallback, fatAABB)
	}

	// Reset move buffer
	bp.M_moveCount = 0

	// Sort the pair buffer to expose duplicates.
	sort.Sort(PairByLessThan(bp.M_pairBuffer[:bp.M_pairCount]))

	// Send the pairs back to the client.
	i := 0
	for i < bp.M_pairCount {
		primaryPair := bp.M_pairBuffer[i]
		userDataA := bp.M_tree.GetUserData(primaryPair.ProxyIdA)
		userDataB := bp.M_tree.GetUserData(primaryPair.ProxyIdB)

		addPairCallback(userDataA, userDataB)
		i++

		// Skip any duplicate pairs.
		for i < bp.M_pairCount {
			pair := bp.M_pairBuffer[i]
			if pair.ProxyIdA != primaryPair.ProxyIdA || pair.ProxyIdB != primaryPair.ProxyIdB {
				break
			}
			i++
		}
	}

	// // Try to keep the tree balanced.
	// //m_tree.Rebalance(4);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// BroadPhase.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func MakeB2BroadPhase() B2BroadPhase {

	pairCapacity := 16
	moveCapacity := 16

	tree := MakeB2DynamicTree()

	return B2BroadPhase{
		M_tree:       tree,
		M_proxyCount: 0,

		M_pairCapacity: pairCapacity,
		M_pairCount:    0,
		M_pairBuffer:   make([]B2Pair, pairCapacity),

		M_moveCapacity: moveCapacity,
		M_moveCount:    0,
		M_moveBuffer:   make([]int, moveCapacity),
	}
}

func (bp *B2BroadPhase) CreateProxy(aabb B2AABB, userData interface{}) int {
	proxyId := bp.M_tree.CreateProxy(aabb, userData)
	bp.M_proxyCount++
	bp.BufferMove(proxyId)
	return proxyId
}

func (bp *B2BroadPhase) DestroyProxy(proxyId int) {
	bp.UnBufferMove(proxyId)
	bp.M_proxyCount--
	bp.M_tree.DestroyProxy(proxyId)
}

func (bp *B2BroadPhase) MoveProxy(proxyId int, aabb B2AABB, displacement B2Vec2) {
	buffer := bp.M_tree.MoveProxy(proxyId, aabb, displacement)
	if buffer {
		bp.BufferMove(proxyId)
	}
}

func (bp *B2BroadPhase) TouchProxy(proxyId int) {
	bp.BufferMove(proxyId)
}

func (bp *B2BroadPhase) BufferMove(proxyId int) {
	if bp.M_moveCount == bp.M_moveCapacity {
		bp.M_moveBuffer = append(bp.M_moveBuffer, make([]int, bp.M_moveCapacity)...)
		bp.M_moveCapacity *= 2
	}

	bp.M_moveBuffer[bp.M_moveCount] = proxyId
	bp.M_moveCount++
}

func (bp *B2BroadPhase) UnBufferMove(proxyId int) {
	for i := 0; i < bp.M_moveCount; i++ {
		if bp.M_moveBuffer[i] == proxyId {
			bp.M_moveBuffer[i] = E_nullProxy
		}
	}
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
func (bp *B2BroadPhase) QueryCallback(proxyId int) bool {

	// A proxy cannot form a pair with itself.
	if proxyId == bp.M_queryProxyId {
		return true
	}

	// Grow the pair buffer as needed.
	if bp.M_pairCount == bp.M_pairCapacity {
		bp.M_pairBuffer = append(bp.M_pairBuffer, make([]B2Pair, bp.M_pairCapacity)...)
		bp.M_pairCapacity *= 2
	}

	bp.M_pairBuffer[bp.M_pairCount].ProxyIdA = MinInt(proxyId, bp.M_queryProxyId)
	bp.M_pairBuffer[bp.M_pairCount].ProxyIdB = MaxInt(proxyId, bp.M_queryProxyId)
	bp.M_pairCount++

	return true
}

func (bp *B2BroadPhase) Query(callback B2TreeQueryCallback, aabb B2AABB) {
	bp.M_tree.Query(callback, aabb)
}

func (bp *B2BroadPhase) RayCast(callback B2TreeRayCastCallback, input B2RayCastInput) {
	bp.M_tree.RayCast(callback, input)
}

func (bp *B2BroadPhase) ShiftOrigin(newOrigin B2Vec2) {
	bp.M_tree.ShiftOrigin(newOrigin)
}
