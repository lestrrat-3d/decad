package decad

import (
	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/units"
)

// This file is agent-docs/sweep-design.md §16's mitred polyline sweep: the two
// options that select it, the gates Table SM states ahead of the build, the
// payload that records it, its placement, and its exact-restatement
// tessellation (Table DM rows DM2 and DM7). internal/sweepmitre constructs
// §16.3's exact sections; sweep_mitre_build.go adapts them to Table BM's
// topology and §16.6's four readings.

// WithMitredJoins admits a LineTo-only path whose internal joins are corners:
// the two spans meeting at a join are each cut on one join plane and share the
// section polygon on it (agent-docs/sweep-design.md §16). The profile must be made
// of straight lines only. Passing it twice is ErrDegenerate.
func WithMitredJoins() SweepOption {
	return sweepOptionValue{featureoption.WithMitredJoins()}
}

// WithSectionScale states one dimensionless factor per path segment, in path
// order: factors[k] is the ratio of the section at the END of span k to the
// authored profile, so a branch whose radius is r_k at path point k passes
// r_k / r_0. Each factor must be a positive, finite Dimensionless value, and
// the count must equal the path's segment count. On a path of two or more
// spans it requires [WithMitredJoins] (agent-docs/sweep-design.md §16.2).
func WithSectionScale(factors ...units.Value) SweepOption {
	return sweepOptionValue{featureoption.WithSectionScale(factors...)}
}
