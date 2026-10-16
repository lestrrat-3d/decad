package meshbool

import "sync"

var validationOwnerCensus struct {
	sync.Mutex
	active                                        bool
	Calls, Eligible, OverCap, Empty, MaxTriangles uint64
	Histogram                                     [16]uint64
	Samples                                       [32]int
}

// ValidationOwnerCensusBegin is disposable instrumentation, never production API.
func ValidationOwnerCensusBegin() {
	c := &validationOwnerCensus
	c.Lock()
	defer c.Unlock()
	c.active = true
	c.Calls = 0
	c.Eligible = 0
	c.OverCap = 0
	c.Empty = 0
	c.MaxTriangles = 0
	c.Histogram = [16]uint64{}
	c.Samples = [32]int{}
}
func validationRecordOwnerRequest(n int) {
	c := &validationOwnerCensus
	c.Lock()
	defer c.Unlock()
	if !c.active {
		return
	}
	if c.Calls < 32 {
		c.Samples[c.Calls] = n
	}
	c.Calls++
	if n == 0 {
		c.Empty++
	} else if n <= 8192 {
		c.Eligible++
	} else {
		c.OverCap++
	}
	if uint64(n) > c.MaxTriangles {
		c.MaxTriangles = uint64(n)
	}
	bin := 0
	for x := n; x > 1 && bin < 15; x >>= 1 {
		bin++
	}
	c.Histogram[bin]++
}
func ValidationOwnerCensusEnd() (uint64, uint64, uint64, uint64, uint64, [16]uint64, [32]int) {
	c := &validationOwnerCensus
	c.Lock()
	defer c.Unlock()
	c.active = false
	return c.Calls, c.Eligible, c.OverCap, c.Empty, c.MaxTriangles, c.Histogram, c.Samples
}
