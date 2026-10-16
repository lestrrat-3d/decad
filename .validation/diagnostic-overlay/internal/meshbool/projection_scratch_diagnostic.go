package meshbool

import "sync"

// This file exists only in the temporary diagnostic overlay.
var projectionScratchDiagnostic struct {
	sync.Mutex
	value ProjectionScratchCounts
}

type ProjectionScratchCounts struct {
	Calls, Regions, Vertices, CopiedPoints, ReusablePoints uint64
	MaxRegions, MaxVertices                                int
	// Vertex buckets: <=8,16,32,64,128,256,512,1024,larger.
	// Region buckets: <=1,2,4,8,16,32,larger.
	Histogram [9][7]uint64
}

func projectionBucket(n int, limits []int) int {
	for i, limit := range limits {
		if n <= limit {
			return i
		}
	}
	return len(limits)
}
func recordProjectionScratchDiagnostic(vertices, regions int) {
	projectionScratchDiagnostic.Lock()
	defer projectionScratchDiagnostic.Unlock()
	d := &projectionScratchDiagnostic.value
	d.Calls++
	d.Regions += uint64(regions)
	d.Vertices += uint64(vertices)
	d.CopiedPoints += uint64(vertices) * uint64(regions)
	if regions > 1 {
		d.ReusablePoints += uint64(vertices) * uint64(regions-1)
	}
	d.MaxRegions = max(d.MaxRegions, regions)
	d.MaxVertices = max(d.MaxVertices, vertices)
	d.Histogram[projectionBucket(vertices, []int{8, 16, 32, 64, 128, 256, 512, 1024})][projectionBucket(regions, []int{1, 2, 4, 8, 16, 32})]++
}
func ResetProjectionScratchDiagnostic() {
	projectionScratchDiagnostic.Lock()
	defer projectionScratchDiagnostic.Unlock()
	projectionScratchDiagnostic.value = ProjectionScratchCounts{}
}
func ProjectionScratchDiagnosticSnapshot() ProjectionScratchCounts {
	projectionScratchDiagnostic.Lock()
	defer projectionScratchDiagnostic.Unlock()
	return projectionScratchDiagnostic.value
}
