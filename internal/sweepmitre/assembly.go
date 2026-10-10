package sweepmitre

import "fmt"

const (
	SlotCapStart = 0
	SlotCapEnd   = 1
	SlotWall0    = 2
)

// Assembly holds the mitred sweep's triangles and their face slots.
// Face slots begin with the two caps, then walls in span, loop, segment order.
type Assembly struct {
	Tris          [][3]int
	TriFace       []int
	Roles         []string
	Walls         int
	CapStartCount int
	Reversed      bool
	Spans         int
	Stride        int
	LoopIdx       [][]int
}

func (a Assembly) At(k, v int) int { return k*a.Stride + v }

// Wall is the wall index of span k over the segment starting at section vertex v.
func (a Assembly) Wall(k, v int) int { return k*a.Stride + v }

// Assemble emits each wall's two triangles, then the start and end caps.
// The end cap reuses the start cap's triangulation under the section map.
func Assemble(c Construction, startRole, endRole string) Assembly {
	a := Assembly{
		Roles:   []string{startRole, endRole},
		Spans:   len(c.Sections) - 1,
		Stride:  len(c.Sections[0]),
		LoopIdx: c.LoopIdx,
	}
	for k := range a.Spans {
		for i, idx := range c.LoopIdx {
			m := len(idx)
			for j := range m {
				slot := len(a.Roles)
				a.Roles = append(a.Roles, fmt.Sprintf("side(%d,%d,%d)", k, i, j))
				b0, b1 := a.At(k, idx[j]), a.At(k, idx[(j+1)%m])
				t0, t1 := a.At(k+1, idx[j]), a.At(k+1, idx[(j+1)%m])
				a.Tris = append(a.Tris, [3]int{b0, b1, t1}, [3]int{b0, t1, t0})
				a.TriFace = append(a.TriFace, slot, slot)
			}
		}
	}
	a.Walls = len(a.Tris)
	for _, t := range c.CapTris {
		a.Tris = append(a.Tris, [3]int{a.At(0, t[0]), a.At(0, t[2]), a.At(0, t[1])})
		a.TriFace = append(a.TriFace, SlotCapStart)
	}
	a.CapStartCount = len(c.CapTris)
	for _, t := range c.CapTris {
		a.Tris = append(a.Tris, [3]int{a.At(a.Spans, t[0]), a.At(a.Spans, t[1]), a.At(a.Spans, t[2])})
		a.TriFace = append(a.TriFace, SlotCapEnd)
	}
	return a
}
