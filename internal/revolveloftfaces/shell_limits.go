package revolveloftfaces

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/sketch"
)

// VerticalOutcome classifies a constant-X cross section of a trimmed LoftFaces
// source against a blank side.
type VerticalOutcome uint8

const (
	VerticalRefusal VerticalOutcome = iota
	VerticalMiss
	VerticalContact
)

func RefFaceIndex(p facetedrecord.Payload, want provenance.Ref, faceCount int) (int, bool) {
	if len(p.Src) != len(p.Tris) || len(p.FaceOf) != len(p.Tris) {
		return -1, false
	}
	index := -1
	for i, group := range p.Src {
		if group < 0 || group >= len(p.Groups) {
			return -1, false
		}
		if slices.Contains(p.Groups[group].Origins, want) {
			if p.FaceOf[i] < 0 || p.FaceOf[i] >= faceCount ||
				index >= 0 && index != p.FaceOf[i] {
				return -1, false
			}
			index = p.FaceOf[i]
		}
	}
	return index, true
}

// The station group labels identify source chords. A complete cyclic run
// contains every chord of this Sketch arc and its next shared endpoint.
func CompleteArcRun(trim *loftFacesTrimRecord, segment int) ([]int, bool) {
	if trim == nil || trim.Source == nil || segment < 0 ||
		segment >= len(trim.Source.Profile.Outer.Segments) ||
		len(trim.StationGroup) != len(trim.Source.SourceStation) ||
		len(trim.StationGroup) < 3 {
		return nil, false
	}
	groups := trim.StationGroup
	start, runs := -1, 0
	for index, group := range groups {
		if group == segment && groups[(index+len(groups)-1)%len(groups)] != segment {
			start, runs = index, runs+1
		}
	}
	if runs != 1 {
		return nil, false
	}
	stations := make([]int, 0, len(groups)+1)
	for index := start; groups[index] == segment; index = (index + 1) % len(groups) {
		stations = append(stations, index)
		if len(stations) >= len(groups) {
			return nil, false
		}
	}
	return append(stations, (stations[len(stations)-1]+1)%len(groups)), true
}

// The Cut keeps the strict outside of its cone. Intersect keeps the strict
// inside of its cone. On an X-constant plane these become radial limits.
func VerticalLimits(trim *loftFacesTrimRecord,
	section revolveSectionSpan, x *big.Rat) (*big.Rat, *big.Rat, VerticalOutcome) {
	if trim == nil || trim.Toe == nil || trim.Heel == nil || x == nil ||
		trim.Toe.Apex == nil || trim.Heel.Apex == nil ||
		trim.Toe.Far == nil || trim.Heel.Far == nil ||
		trim.Toe.Slope == nil || trim.Heel.Slope == nil ||
		trim.Toe.Slope.Sign() <= 0 || trim.Heel.Slope.Sign() <= 0 ||
		x.Cmp(trim.Toe.Far) <= 0 || x.Cmp(trim.Heel.Far) <= 0 ||
		x.Cmp(trim.Heel.Apex) >= 0 {
		return nil, nil, VerticalRefusal
	}
	lower, upper := proofarith.FloatRat(section.Start.V), proofarith.FloatRat(section.End.V)
	if lower.Cmp(upper) > 0 {
		lower, upper = upper, lower
	}
	if x.Cmp(trim.Toe.Apex) < 0 {
		toe := proofarith.QuoRat(new(big.Rat), proofarith.SubRat(new(big.Rat), trim.Toe.Apex, x), trim.Toe.Slope)
		if lower.Cmp(toe) < 0 {
			lower = toe
		}
	}
	heel := proofarith.QuoRat(new(big.Rat), proofarith.SubRat(new(big.Rat), trim.Heel.Apex, x), trim.Heel.Slope)
	if upper.Cmp(heel) > 0 {
		upper = heel
	}
	if lower.Sign() < 0 || upper.Sign() < 0 || lower.Cmp(upper) == 0 {
		return lower, upper, VerticalRefusal
	}
	if lower.Cmp(upper) > 0 {
		return lower, upper, VerticalMiss
	}
	return lower, upper, VerticalContact
}

func OriginsMatch(actual, recorded []provenance.Ref) bool {
	if len(actual) != len(recorded) {
		return false
	}
	for i := range actual {
		if actual[i] != recorded[i] {
			return false
		}
	}
	return true
}

func ToolBoundaryOutward(edge sketch.BoundaryEdge,
	limit loftface.VerticalLimit) (bool, bool) {
	if edge.Partial || limit.Before*limit.After != -1 {
		return false, false
	}
	before, after := limit.Before, limit.After
	if edge.Reversed {
		before, after = after, before
	}
	return before < 0 && after > 0, true
}
