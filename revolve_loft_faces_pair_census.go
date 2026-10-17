package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// A pair verdict is a source answer for one live blank and trimmed face.
// A candidate still needs a shared mesh, topology and volume proof.
type revolveLoftFacesPairVerdict uint8

const (
	revolveLoftFacesPairRefusal revolveLoftFacesPairVerdict = iota
	revolveLoftFacesPairMiss
	revolveLoftFacesPairSourceCandidate
	revolveLoftFacesPairToolArc
)

type revolveLoftFacesFacePair struct {
	blankFace, trimmedFace *Face
	sectionIndex           int
	verdict                revolveLoftFacesPairVerdict
	sourceCells            []revolveLoftFacesSourceCell
	verticalCells          []revolveLoftFacesVerticalCell
	toolArc                *revolveLoftFacesToolCapOccupiedArc
}

type revolveLoftFacesPairCensus struct {
	manifest *revolveLoftFacesContactManifest
	pairs    []revolveLoftFacesFacePair
	// ownArcs holds each occupied arc this census derived, with a private
	// deep copy taken at derivation. The copies are never handed out.
	ownArcs []revolveLoftFacesCensusArc
}

// revolveLoftFacesCensusArc is an occupied arc this census derived and a
// private deep copy of it taken at derivation. The copy is never handed out.
type revolveLoftFacesCensusArc struct {
	arc     *revolveLoftFacesToolCapOccupiedArc
	derived revolveLoftFacesToolCapOccupiedArc
}

// The census classifies every actual face pair from live producers. It does
// not admit Union: a source candidate is only a possible contact curve, and
// an occupied tool arc still needs exact common held vertices and bounds.
func revolveLoftFacesCensus(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (*revolveLoftFacesPairCensus, bool, error) {
	if ctx == nil || proof == nil || proof.blank == nil || proof.tooth == nil {
		return nil, false, nil
	}
	ctx = revolveloftfaces.RevolveLoftFacesWithMeridianMemo(ctx)
	manifest, ok, err := revolveLoftFacesAllSideManifest(ctx, proof)
	if err != nil || !ok {
		return nil, false, err
	}
	toolSides, ok := revolveLoftFacesToolCapSidesOf(proof)
	if !ok {
		return nil, false, nil
	}
	blankFaces, trimmedFaces := proof.blank.Faces(), proof.tooth.Faces()
	if len(blankFaces) == 0 || len(trimmedFaces) == 0 ||
		len(blankFaces)*len(trimmedFaces) > 1<<16 {
		return nil, false, nil
	}
	payload, ok := proof.tooth.n.Payload.(facetedPayload)
	if !ok || payload.LoftFacesTrim != proof.trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, proof.trim) {
		return nil, false, nil
	}
	wallFaces := make(map[*Face]struct{})
	for _, band := range proof.sourceRefs {
		for _, ref := range band {
			index, valid := revolveLoftFacesRefFaceIndex(payload, ref, len(trimmedFaces))
			face, unique := revolveLoftFacesUniqueOriginFace(proof.tooth, ref)
			if !valid || !unique || (index >= 0) != (face != nil) ||
				index >= 0 && trimmedFaces[index] != face {
				return nil, false, nil
			}
			if face != nil {
				wallFaces[face] = struct{}{}
			}
		}
	}
	_, toeWall := wallFaces[proof.toeFace]
	_, heelWall := wallFaces[proof.heelFace]
	if toeWall || heelWall {
		return nil, false, nil
	}
	seenTrimmed := make(map[*Face]struct{}, len(trimmedFaces))
	for _, face := range trimmedFaces {
		_, duplicate := seenTrimmed[face]
		_, wall := wallFaces[face]
		if face == nil || duplicate ||
			face != proof.toeFace && face != proof.heelFace && !wall {
			return nil, false, fmt.Errorf("%w: trimmed LoftFaces face has no unique live source role", ErrUnsupported)
		}
		seenTrimmed[face] = struct{}{}
	}
	sectionOf := make(map[*topo.Face]int)
	for index, span := range proof.section.Spans {
		if span.Face == nil {
			continue
		}
		if _, duplicate := sectionOf[span.Face]; duplicate {
			return nil, false, nil
		}
		sectionOf[span.Face] = index
	}
	if len(sectionOf) != len(blankFaces) {
		return nil, false, nil
	}
	seenBlank := make(map[*Face]struct{}, len(blankFaces))
	for _, face := range blankFaces {
		_, duplicate := seenBlank[face]
		if face == nil || duplicate {
			return nil, false, nil
		}
		seenBlank[face] = struct{}{}
		if _, found := sectionOf[faceNode(face)]; !found {
			return nil, false, nil
		}
	}
	nonvertical := make(map[int]revolveLoftFacesNonverticalSide)
	for _, side := range manifest.nonvertical {
		nonvertical[side.sectionIndex] = side
	}
	vertical := make(map[int]revolveLoftFacesVerticalEntry)
	for _, side := range manifest.vertical {
		vertical[side.sectionIndex] = side
	}
	circles := make(map[[2]*Face]revolveLoftFacesToolCapCircle)
	for _, circle := range toolSides.circles {
		key := [2]*Face{circle.blankFace, circle.capFace}
		if _, duplicate := circles[key]; duplicate {
			return nil, false, nil
		}
		circles[key] = circle
	}
	result := &revolveLoftFacesPairCensus{manifest: manifest,
		pairs: make([]revolveLoftFacesFacePair, 0, len(blankFaces)*len(trimmedFaces))}
	for _, blankFace := range blankFaces {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		sectionIndex, found := sectionOf[faceNode(blankFace)]
		if !found || sectionIndex >= len(manifest.sideOutcome) {
			return nil, false, nil
		}
		for _, trimmedFace := range trimmedFaces {
			pair := revolveLoftFacesFacePair{blankFace: blankFace,
				trimmedFace: trimmedFace, sectionIndex: sectionIndex}
			if trimmedFace == proof.toeFace || trimmedFace == proof.heelFace {
				key := [2]*Face{blankFace, trimmedFace}
				circle, possible := circles[key]
				_, separated := toolSides.separated[key]
				if possible == separated {
					return nil, false, nil
				}
				if separated || circle.sourceMiss {
					pair.verdict = revolveLoftFacesPairMiss
				} else {
					if circle.tool != 1 {
						return nil, false, nil
					}
					var arc *revolveLoftFacesToolCapOccupiedArc
					var certified bool
					var arcErr error
					span := proof.section.Spans[circle.span]
					if span.Start.U == span.End.U {
						arc, certified, arcErr = revolveLoftFacesHeelVerticalOccupiedArc(ctx, proof, circle)
					} else {
						arc, certified, arcErr = revolveLoftFacesHeelSlopedOccupiedArc(ctx, proof, circle)
					}
					if arcErr != nil || !certified || arc == nil {
						return nil, false, arcErr
					}
					pair.verdict, pair.toolArc = revolveLoftFacesPairToolArc, arc
					result.ownArcs = append(result.ownArcs, revolveLoftFacesCensusArc{
						arc: arc, derived: revolveLoftFacesCloneToolCapOccupiedArc(arc)})
				}
			} else {
				switch manifest.sideOutcome[sectionIndex] {
				case revolveLoftFacesSideMiss:
					pair.verdict = revolveLoftFacesPairMiss
				case revolveLoftFacesSideSourceCandidate:
					if side, found := nonvertical[sectionIndex]; found {
						for _, cell := range side.cells {
							if cell.face != trimmedFace {
								continue
							}
							pair.sourceCells = append(pair.sourceCells, cell)
							switch cell.mapped.Side {
							case loftface.SideCrossed, revolveloftfaces.RevolveLoftFacesJointContact:
								pair.verdict = revolveLoftFacesPairSourceCandidate
							case loftface.SideInside, loftface.SideOutside,
								revolveloftfaces.RevolveLoftFacesAxialMiss:
							default:
								return nil, false, nil
							}
						}
						if len(pair.sourceCells) == 0 {
							return nil, false, nil
						}
					} else if side, found := vertical[sectionIndex]; found && side.side != nil {
						for _, cell := range side.side.cells {
							if cell.face == trimmedFace && len(cell.retained) > 0 {
								pair.verticalCells = append(pair.verticalCells, cell)
							}
						}
						if len(pair.verticalCells) > 0 {
							pair.verdict = revolveLoftFacesPairSourceCandidate
						}
					} else {
						return nil, false, nil
					}
					if pair.verdict == revolveLoftFacesPairRefusal {
						pair.verdict = revolveLoftFacesPairMiss
					}
				default:
					return nil, false, nil
				}
			}
			result.pairs = append(result.pairs, pair)
		}
	}
	return result, len(result.pairs) == len(blankFaces)*len(trimmedFaces), nil
}
