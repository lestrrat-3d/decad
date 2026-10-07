package sectionrecord

import (
	"math"
	"reflect"
)

// IdenticalRecord reports whether two recorded values have the same dynamic
// type, shape, fields and float bits. A zero of the other sign is a mismatch.
// It refuses any kind the traversal does not know, so a new field cannot
// silently make two different records match.
//
// This exact cache guard uses reflection because a hand-written comparator
// could omit a field added to a record variant. It only reads fields, including
// unexported fields of nested values such as units.Value. An unknown shape
// costs the caller a cached read; accepting a wrong match would publish the
// other record's geometry.
func IdenticalRecord(a, b any) bool {
	return identicalRecordValue(reflect.ValueOf(a), reflect.ValueOf(b))
}

// identicalRecordValue also handles the zero reflect.Value produced by a nil
// interface; it matches only another zero value.
func identicalRecordValue(a, b reflect.Value) bool {
	if !a.IsValid() || !b.IsValid() {
		return a.IsValid() == b.IsValid()
	}
	if a.Type() != b.Type() {
		return false
	}
	switch a.Kind() { //nolint:exhaustive // an unhandled kind is a mismatch, by the doc comment above.
	case reflect.Bool:
		return a.Bool() == b.Bool()
	case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		return a.Int() == b.Int()
	case reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64, reflect.Uintptr:
		return a.Uint() == b.Uint()
	case reflect.Float32, reflect.Float64:
		// Float() widens a float32 exactly, so one comparison serves both.
		return math.Float64bits(a.Float()) == math.Float64bits(b.Float())
	case reflect.String:
		return a.String() == b.String()
	case reflect.Struct:
		for i := range a.NumField() {
			// Field reads an unexported field read-only, which is all this
			// traversal ever does — units.Value's own magnitude and unit are
			// unexported and are compared here like any other field.
			if !identicalRecordValue(a.Field(i), b.Field(i)) {
				return false
			}
		}
		return true
	case reflect.Slice, reflect.Array:
		if a.Len() != b.Len() {
			return false
		}
		for i := range a.Len() {
			if !identicalRecordValue(a.Index(i), b.Index(i)) {
				return false
			}
		}
		return true
	case reflect.Interface, reflect.Pointer:
		if a.IsNil() || b.IsNil() {
			return a.IsNil() && b.IsNil()
		}
		return identicalRecordValue(a.Elem(), b.Elem())
	default:
		return false
	}
}
