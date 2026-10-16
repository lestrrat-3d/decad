package prismpattern

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// FilletRingCut lists the ring stations a contact consumes. Positions run from
// the source root through the last ring station consumed by a contact. The
// next station, Far, remains on the circular wall. Both caps use these same
// outline positions and must have matching XY stations.
type FilletRingCut struct {
	Positions []int
	Far       int
}

// FilletRingCutOf certifies the held and source ring stations a fillet
// contact consumes, or returns ErrUnsupported.
func FilletRingCutOf(rec *facetedrecord.PrismPatternJoinRecord,
	contact FilletContact) (FilletRingCut, error) {
	var empty FilletRingCut
	outline := rec.Outlines[0]
	if len(outline) < 4 || len(rec.Outlines[1]) != len(outline) {
		return empty, fmt.Errorf("%w: joined-prism fillet lost its paired ring outlines", decaderr.ErrUnsupported)
	}
	part := rec.Parts[contact.part]
	root, limit, direction := part.Vertex[0][part.RootStart], 0, 1
	if contact.station == 0 {
		root = part.Vertex[0][0]
		previous := rec.Parts[(contact.part+len(rec.Parts)-1)%len(rec.Parts)]
		limit = previous.Vertex[0][previous.RootStart]
		direction = -1
	} else {
		next := rec.Parts[(contact.part+1)%len(rec.Parts)]
		limit = next.Vertex[0][0]
	}
	find := func(index int) int {
		for i, found := range outline {
			if found == index {
				return i
			}
		}
		return -1
	}
	position, limitPosition := find(root), find(limit)
	if position < 0 || limitPosition < 0 || position == limitPosition {
		return empty, fmt.Errorf("%w: joined-prism fillet has no bounded source ring gap", decaderr.ErrUnsupported)
	}
	point := func(position int) (IVPoint, bool) {
		bottom := rec.Mesh.Verts[rec.Outlines[0][position]]
		top := rec.Mesh.Verts[rec.Outlines[1][position]]
		if bottom.X != top.X || bottom.Y != top.Y ||
			bottom.Z != rec.Source.Blank.Z0 || top.Z != rec.Source.Blank.Z1 {
			return IVPoint{}, false
		}
		return IVPointOf(proofarith.FloatRat(bottom.X), proofarith.FloatRat(bottom.Y)), true
	}
	// A held station can sit on the other side of a contact from its true
	// circle station. Check both orders before shortening the source chord.
	trueRoots := make(map[int]IVPoint, 2*len(rec.Parts))
	for _, sourcePart := range rec.Parts {
		trueRoots[sourcePart.Vertex[0][0]] = exactPlacedPoint(sourcePart,
			rec.Source.RootArc.Start)
		trueRoots[sourcePart.Vertex[0][sourcePart.RootStart]] = exactPlacedPoint(sourcePart,
			rec.Source.RootArc.End)
	}
	sourcePoint := func(position int, held IVPoint) IVPoint {
		if root, ok := trueRoots[outline[position]]; ok {
			return root
		}
		widen := proofarith.FloatRat(rec.RingRound)
		return IVPoint{
			X: proofbound.IntervalWiden(held.X, widen),
			Y: proofbound.IntervalWiden(held.Y, widen),
		}
	}
	cross := func(a, b IVPoint) proofbound.RatInterval {
		return proofbound.IntervalSub(proofbound.IntervalMul(a.X, b.Y),
			proofbound.IntervalMul(a.Y, b.X))
	}
	cut := FilletRingCut{}
	for step := 0; step < len(outline) && step < 512; step++ {
		next := (position + direction + len(outline)) % len(outline)
		if next == limitPosition {
			break
		}
		currentIV, currentOK := point(position)
		nextIV, nextOK := point(next)
		if !currentOK || !nextOK {
			return empty, fmt.Errorf("%w: joined-prism fillet ring stations do not pair", decaderr.ErrUnsupported)
		}
		q := contact.circleIV
		// Status 1 brackets Q in this cell; status 2 places Q farther
		// along the same ordered gap. Zero has no strict exact proof.
		classify := func(current, next IVPoint) int {
			if direction < 0 {
				if cross(next, current).Lo.Sign() <= 0 || cross(q, current).Lo.Sign() <= 0 {
					return 0
				}
				if cross(next, q).Lo.Sign() > 0 {
					return 1
				}
				if cross(q, next).Lo.Sign() > 0 {
					return 2
				}
				return 0
			}
			if cross(current, next).Lo.Sign() <= 0 || cross(current, q).Lo.Sign() <= 0 {
				return 0
			}
			if cross(q, next).Lo.Sign() > 0 {
				return 1
			}
			if cross(next, q).Lo.Sign() > 0 {
				return 2
			}
			return 0
		}
		heldStatus := classify(currentIV, nextIV)
		trueStatus := classify(sourcePoint(position, currentIV), sourcePoint(next, nextIV))
		if heldStatus == 0 || heldStatus != trueStatus {
			break
		}
		if heldStatus == 1 {
			cut.Positions = append(cut.Positions, position)
			cut.Far = next
			return cut, nil
		}
		cut.Positions = append(cut.Positions, position)
		position = next
	}
	return empty, fmt.Errorf("%w: joined-prism fillet contact has no certified ring bracket", decaderr.ErrUnsupported)
}
