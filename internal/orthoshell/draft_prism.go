package orthoshell

import (
	"errors"

	"github.com/lestrrat-3d/r3"
)

// PreparedPrism is the section frame and sweep distance for drafting every wall
// of a straight prism about one of its caps.
type PreparedPrism struct {
	Loop         [][2]float64
	UAxis, VAxis int
	Distance     float64
}

// PrepareDraftPrism admits a straight prism and extracts its section without
// depending on Decad's Body or Face handles. selectedFaces contains indexes into
// capFaces; every non-cap face must appear exactly once.
func PrepareDraftPrism(
	verts []r3.Vec,
	tris [][3]int,
	axis int,
	neutralLevel float64,
	capFaces []bool,
	selectedFaces []int,
) (PreparedPrism, bool, error) {
	coord := func(v r3.Vec) float64 {
		return [3]float64{v.X, v.Y, v.Z}[axis]
	}
	lo, hi := coord(verts[0]), coord(verts[0])
	for _, v := range verts {
		c := coord(v)
		switch {
		case c == lo || c == hi:
		case lo == hi:
			lo, hi = min(lo, c), max(hi, c)
		default:
			return PreparedPrism{}, false, nil
		}
	}
	if lo == hi || (neutralLevel != lo && neutralLevel != hi) {
		return PreparedPrism{}, false, nil
	}

	caps := 0
	walls := make(map[int]bool, len(capFaces))
	for i, isCap := range capFaces {
		if isCap {
			caps++
		} else {
			walls[i] = true
		}
	}
	if caps != 2 || len(selectedFaces) != len(walls) {
		return PreparedPrism{}, false, nil
	}
	for _, face := range selectedFaces {
		if !walls[face] {
			return PreparedPrism{}, false, nil
		}
		delete(walls, face)
	}

	sweep := 1.0
	if neutralLevel == hi {
		sweep = -1
	}
	a1, a2 := (axis+1)%3, (axis+2)%3
	uAxis, vAxis := a1, a2
	if sweep < 0 {
		uAxis, vAxis = a2, a1
	}
	loops, err := SectionLoops(verts, tris, axis, (lo+hi)/2, uAxis, vAxis)
	if err != nil {
		if errors.Is(err, ErrUnsupported) {
			return PreparedPrism{}, false, nil
		}
		return PreparedPrism{}, false, err
	}
	if len(loops) != 1 || len(loops[0]) < 3 {
		return PreparedPrism{}, false, nil
	}
	return PreparedPrism{
		Loop: loops[0], UAxis: uAxis, VAxis: vAxis, Distance: hi - lo,
	}, true, nil
}
