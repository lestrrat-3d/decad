// Package exactbool holds the exact predicates and implicit points of the polyhedral Boolean kernel
// (agent-docs/exact-boolean-design.md). Every sign it returns is the sign of the exact value: a float evaluation
// answers when its error bound allows, and the dyadic arithmetic of internal/proof answers otherwise. No decision reads
// a rounded coordinate.
package exactbool
