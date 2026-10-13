package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/prismpattern"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// positions run from the source root through the last ring station consumed
// by a contact. The next station, far, remains on the circular wall. Both
// caps use these same outline positions and must have matching XY stations.
type prismPatternFilletRingCut struct {
	positions []int
	far       int
}

func prismPatternFilletRingCutOf(rec *prismPatternJoinRecord,
	contact prismPatternFilletContact) (prismPatternFilletRingCut, error) {
	var empty prismPatternFilletRingCut
	outline := rec.outlines[0]
	if len(outline) < 4 || len(rec.outlines[1]) != len(outline) {
		return empty, fmt.Errorf("%w: joined-prism fillet lost its paired ring outlines", ErrUnsupported)
	}
	part := rec.parts[contact.part]
	root, limit, direction := part.vertex[0][part.rootStart], 0, 1
	if contact.station == 0 {
		root = part.vertex[0][0]
		previous := rec.parts[(contact.part+len(rec.parts)-1)%len(rec.parts)]
		limit = previous.vertex[0][previous.rootStart]
		direction = -1
	} else {
		next := rec.parts[(contact.part+1)%len(rec.parts)]
		limit = next.vertex[0][0]
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
		return empty, fmt.Errorf("%w: joined-prism fillet has no bounded source ring gap", ErrUnsupported)
	}
	point := func(position int) (prismpattern.IVPoint, bool) {
		bottom := rec.mesh.verts[rec.outlines[0][position]]
		top := rec.mesh.verts[rec.outlines[1][position]]
		if bottom.X != top.X || bottom.Y != top.Y ||
			bottom.Z != rec.source.blank.Z0 || top.Z != rec.source.blank.Z1 {
			return prismpattern.IVPoint{}, false
		}
		return prismpattern.IVPointOf(proofarith.FloatRat(bottom.X), proofarith.FloatRat(bottom.Y)), true
	}
	// A held station can sit on the other side of a contact from its true
	// circle station. Check both orders before shortening the source chord.
	trueRoots := make(map[int]prismpattern.IVPoint, 2*len(rec.parts))
	for _, sourcePart := range rec.parts {
		trueRoots[sourcePart.vertex[0][0]] = prismPatternExactPlacedPoint(sourcePart,
			rec.source.rootArc.Start)
		trueRoots[sourcePart.vertex[0][sourcePart.rootStart]] = prismPatternExactPlacedPoint(sourcePart,
			rec.source.rootArc.End)
	}
	sourcePoint := func(position int, held prismpattern.IVPoint) prismpattern.IVPoint {
		if root, ok := trueRoots[outline[position]]; ok {
			return root
		}
		widen := proofarith.FloatRat(rec.ringRound)
		return prismpattern.IVPoint{
			X: proofbound.IntervalWiden(held.X, widen),
			Y: proofbound.IntervalWiden(held.Y, widen),
		}
	}
	cross := func(a, b prismpattern.IVPoint) proofbound.RatInterval {
		return proofbound.IntervalSub(proofbound.IntervalMul(a.X, b.Y),
			proofbound.IntervalMul(a.Y, b.X))
	}
	cut := prismPatternFilletRingCut{}
	for step := 0; step < len(outline) && step < 512; step++ {
		next := (position + direction + len(outline)) % len(outline)
		if next == limitPosition {
			break
		}
		currentIV, currentOK := point(position)
		nextIV, nextOK := point(next)
		if !currentOK || !nextOK {
			return empty, fmt.Errorf("%w: joined-prism fillet ring stations do not pair", ErrUnsupported)
		}
		q := contact.circleIV
		// Status 1 brackets Q in this cell; status 2 places Q farther
		// along the same ordered gap. Zero has no strict exact proof.
		classify := func(current, next prismpattern.IVPoint) int {
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
			cut.positions = append(cut.positions, position)
			cut.far = next
			return cut, nil
		}
		cut.positions = append(cut.positions, position)
		position = next
	}
	return empty, fmt.Errorf("%w: joined-prism fillet contact has no certified ring bracket", ErrUnsupported)
}
