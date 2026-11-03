//go:build !race

package revolvemesh

import "github.com/lestrrat-3d/decad/internal/proof"

// RevDown is proof.NextDown(x) and RevUp is proof.NextUp(x). The plain build's
// proof steps are bit operations small enough to inline here; the race build's
// float arithmetic version in tessellate_revolve_step.go returns the same bits
// for every x.
func RevDown(x float64) float64 { return proof.NextDown(x) }

func RevUp(x float64) float64 { return proof.NextUp(x) }
