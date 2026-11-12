package explodeblock

import "math"

// grid buckets the projected triangles of one mesh by the square cells their
// boxes touch. Cell (x, y) covers [x*cell, (x+1)*cell) by [y*cell, (y+1)*cell)
// in projection coordinates. Only the cells between the lowest and highest
// cell any triangle touches are stored, densely: a query clamps to them, so
// the cells outside the mesh's shadow cost nothing.
type grid struct {
	cell   float64
	x0, y0 int32 // the cell numbers of the first stored column and row
	nx, ny int
	// start[k]..start[k+1] is the run of items of stored cell k = (x-x0)*ny + (y-y0).
	start []int32
	items []int32 // triangle indices, ascending within each cell
}

// maxCellsPerTri caps the stored cells at this many per triangle (plus a fixed
// allowance), so a long thin shadow cannot make the grid huge.
const (
	maxCellsPerTri = 16
	maxCellsExtra  = 4096
)

// newGrid buckets tris. minU..maxV is the box of the mesh's vertices and only
// sets the cell size: about two cells across per triangle on average.
func newGrid(tris []tri, minU, maxU, minV, maxV float64) grid {
	if len(tris) == 0 {
		return grid{cell: 1}
	}
	area := math.Max(maxU-minU, 1e-6) * math.Max(maxV-minV, 1e-6)
	g := grid{cell: math.Max(2*math.Sqrt(area/float64(len(tris))), 1e-3)}
	loX, hiX, loY, hiY := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	for i := range tris {
		t := &tris[i]
		loX, hiX = math.Min(loX, t.minX), math.Max(hiX, t.maxX)
		loY, hiY = math.Min(loY, t.minY), math.Max(hiY, t.maxY)
	}
	limit := float64(maxCellsPerTri*len(tris) + maxCellsExtra)
	for {
		fx0, fx1 := math.Floor(loX/g.cell), math.Floor(hiX/g.cell)
		fy0, fy1 := math.Floor(loY/g.cell), math.Floor(hiY/g.cell)
		if (fx1-fx0+1)*(fy1-fy0+1) <= limit {
			g.x0, g.y0 = int32(fx0), int32(fy0)
			g.nx, g.ny = int(fx1-fx0)+1, int(fy1-fy0)+1
			break
		}
		g.cell *= 2
	}
	g.start = make([]int32, g.nx*g.ny+1)
	for i := range tris {
		t := &tris[i]
		x0, x1, y0, y1, _ := g.cells(t.minX, t.maxX, t.minY, t.maxY)
		for x := x0; x <= x1; x++ {
			for y := y0; y <= y1; y++ {
				g.start[x*g.ny+y+1]++
			}
		}
	}
	for k := range g.nx * g.ny {
		g.start[k+1] += g.start[k]
	}
	g.items = make([]int32, g.start[g.nx*g.ny])
	fill := make([]int32, g.nx*g.ny)
	copy(fill, g.start)
	for i := range tris {
		t := &tris[i]
		x0, x1, y0, y1, _ := g.cells(t.minX, t.maxX, t.minY, t.maxY)
		for x := x0; x <= x1; x++ {
			for y := y0; y <= y1; y++ {
				k := x*g.ny + y
				g.items[fill[k]] = int32(i)
				fill[k]++
			}
		}
	}
	return g
}

// cells returns the stored columns x0..x1 and rows y0..y1 (0-based) that the
// box [lx, hx] by [ly, hy] touches, and false when it touches none.
func (g *grid) cells(lx, hx, ly, hy float64) (int, int, int, int, bool) {
	x0, x1, okX := g.axis(lx, hx, g.x0, g.nx)
	y0, y1, okY := g.axis(ly, hy, g.y0, g.ny)
	return x0, x1, y0, y1, okX && okY
}

func (g *grid) axis(lo, hi float64, first int32, n int) (int, int, bool) {
	a, b := math.Floor(lo/g.cell), math.Floor(hi/g.cell)
	f, l := float64(first), float64(first)+float64(n-1)
	if !(b >= f) || !(a <= l) {
		return 0, 0, false
	}
	return int(math.Max(a, f) - f), int(math.Min(b, l) - f), true
}

// at returns the triangles bucketed in stored cell (x, y), ascending.
func (g *grid) at(x, y int) []int32 {
	k := x*g.ny + y
	return g.items[g.start[k]:g.start[k+1]]
}
