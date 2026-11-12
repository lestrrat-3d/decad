// Package explodeplan walks the joint graph of a build from a ground part and
// chooses each part's offset for an exploded view (agent-docs/parts-design.md
// §5). It works on plain millimetre boxes and touches no body.
package explodeplan

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// Box is an axis-aligned box.
type Box struct{ Min, Max r3.Vec }

// Centre returns the box centre.
func (b Box) Centre() r3.Vec { return b.Min.Add(b.Max).Scale(0.5) }

// Mode says how a joint gives the direction a part is pulled.
type Mode int

const (
	// ModeCentres has no direction of its own: the part moves along the line
	// from its parent's box centre to its own.
	ModeCentres Mode = iota
	// ModeAxis pulls along Dir in either sense.
	ModeAxis
	// ModeAway pulls A along Dir and B along -Dir: Dir is the direction that
	// takes A away from B.
	ModeAway
)

// Joint joins parts A and B, which index the boxes given to Plan. Source is
// carried to the result untouched.
type Joint struct {
	A, B   int
	Mode   Mode
	Dir    r3.Vec // unit; unused for ModeCentres
	Source int
}

const eps = 1e-9

func shift(b Box, o r3.Vec) Box { return Box{Min: b.Min.Add(o), Max: b.Max.Add(o)} }

// better reports whether choice a beats b: a free sense beats a blocked one,
// then the shorter travel, then the sense whose largest component is positive.
func better(af bool, at float64, am r3.Vec, bf bool, bt float64, bm r3.Vec) bool {
	if af != bf {
		return af
	}
	if math.Abs(at-bt) > eps {
		return at < bt
	}
	return dominantPositive(am) && !dominantPositive(bm)
}

func dominantPositive(v r3.Vec) bool {
	a := [3]float64{v.X, v.Y, v.Z}
	k := 0
	for i := 1; i < 3; i++ {
		if math.Abs(a[i]) > math.Abs(a[k])+eps {
			k = i
		}
	}
	return a[k] > 0
}

// direction returns the unit direction the child moves along and whether the
// opposite sense is allowed too. A zero vector means none.
func direction(boxes []Box, j Joint, parent, child int) (r3.Vec, bool) {
	switch j.Mode {
	case ModeAxis:
		return j.Dir, true
	case ModeAway:
		if child == j.A {
			return j.Dir, false
		}
		return j.Dir.Scale(-1), false
	}
	d := boxes[child].Centre().Sub(boxes[parent].Centre())
	u, ok := d.Normalize()
	if !ok {
		return r3.Vec{}, false
	}
	return u, false
}

// travel is the least distance t >= 0 along unit dir that takes box b clear of
// box p, or +Inf when dir never separates them.
func travel(b, p Box, dir r3.Vec) float64 {
	bl := [3]float64{b.Min.X, b.Min.Y, b.Min.Z}
	bh := [3]float64{b.Max.X, b.Max.Y, b.Max.Z}
	pl := [3]float64{p.Min.X, p.Min.Y, p.Min.Z}
	ph := [3]float64{p.Max.X, p.Max.Y, p.Max.Z}
	d := [3]float64{dir.X, dir.Y, dir.Z}
	t := math.Inf(1)
	for k := range 3 {
		switch {
		case d[k] > eps:
			t = math.Min(t, (ph[k]-bl[k])/d[k])
		case d[k] < -eps:
			t = math.Min(t, (bh[k]-pl[k])/-d[k])
		}
	}
	if t < 0 {
		return 0
	}
	return t
}

// Port returns the point where a joint between boxes a and b attaches when the
// faces say nothing: the centre of their overlap, or the midpoint of the
// shortest segment between them when they do not overlap.
func Port(a, b Box) r3.Vec {
	lo := r3.Vec{X: math.Max(a.Min.X, b.Min.X), Y: math.Max(a.Min.Y, b.Min.Y), Z: math.Max(a.Min.Z, b.Min.Z)}
	hi := r3.Vec{X: math.Min(a.Max.X, b.Max.X), Y: math.Min(a.Max.Y, b.Max.Y), Z: math.Min(a.Max.Z, b.Max.Z)}
	return lo.Add(hi).Scale(0.5)
}

// Meet reports whether two boxes touch or overlap.
func Meet(a, b Box) bool {
	return a.Min.X <= b.Max.X && b.Min.X <= a.Max.X &&
		a.Min.Y <= b.Max.Y && b.Min.Y <= a.Max.Y &&
		a.Min.Z <= b.Max.Z && b.Min.Z <= a.Max.Z
}

// DominantPositive reports whether the largest component of v is positive; ties
// favour the earlier axis.
func DominantPositive(v r3.Vec) bool { return dominantPositive(v) }
