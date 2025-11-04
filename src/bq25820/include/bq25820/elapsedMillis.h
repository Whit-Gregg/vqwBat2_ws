/* Elapsed time types - for easy-to-use measurements of elapsed time
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file elapsedMillis.h
 * @brief Lightweight elapsed time utility types for measuring milliseconds and microseconds.
 *
 * This header defines two convenience classes, elapsedMillis and elapsedMicros, that behave like
 * integers which automatically increase with the passage of time. They are useful for creating
 * non-blocking delays, timeouts, and measuring durations without managing timestamps manually.
 *
 * Example usage:
 * @code{.cpp}
 * elapsedMillis ms;   // starts at 0
 * // ... do some work ...
 * if (ms > 250) {
 *     // 250 ms have elapsed since construction or last assignment
 * }
 *
 * elapsedMicros us(100); // preset to 100 microseconds elapsed
 * // ... later ...
 * unsigned long e = us;  // implicit conversion to elapsed microseconds
 * @endcode
 */

#ifndef elapsedMillis_h
#define elapsedMillis_h
#ifdef __cplusplus


// elapsedMillis acts as an integer which automatically increments 1000 times
// per second. Useful for creating delays, timeouts, or measuring how long an
// operation takes. You can create as many elapsedMillis variables as needed.
// All of them are independent. Any may be written, modified or read at any time.

/**
 * @brief Tracks elapsed time in milliseconds.
 *
 * The class stores an internal snapshot of the system millisecond count at construction
 * or assignment. Reading the value (via implicit conversion to unsigned long) yields the
 * number of milliseconds that have elapsed since that snapshot.
 *
 * Semantics are similar to a stopwatch: assigning a value "val" effectively sets the
 * elapsed time to @c val, and arithmetic operators adjust the stored baseline accordingly.
 *
 * Thread-safety: This class itself is not synchronized. Underlying millis() should be
 * monotonic and safe to call in your environment.
 */
class elapsedMillis
{
private:
	/// Snapshot of system milliseconds at the reference point.
	unsigned long ms;
	/// Returns the system millisecond tick count.
	unsigned long millis() const;
public:
	/// Construct and start at 0 ms elapsed.
	elapsedMillis(void) { ms = millis(); }
	/// Construct with an initial elapsed value in milliseconds.
	elapsedMillis(unsigned long val) { ms = millis() - val; }
	/// Copy-construct from another elapsedMillis.
	elapsedMillis(const elapsedMillis &orig) { ms = orig.ms; }
	/// Implicitly convert to elapsed milliseconds since the reference point.
	operator unsigned long () const { return millis() - ms; }
	/// Assign from another instance (copies the reference point).
	elapsedMillis & operator = (const elapsedMillis &rhs) { ms = rhs.ms; return *this; }
	/// Set current elapsed value to 'val' milliseconds.
	elapsedMillis & operator = (unsigned long val) { ms = millis() - val; return *this; }
	/// Decrease the reported elapsed time by 'val' milliseconds.
	elapsedMillis & operator -= (unsigned long val)      { ms += val ; return *this; }
	/// Increase the reported elapsed time by 'val' milliseconds.
	elapsedMillis & operator += (unsigned long val)      { ms -= val ; return *this; }
	/// Return a copy whose elapsed time is decreased by 'val'.
	elapsedMillis operator - (int val) const           { elapsedMillis r(*this); r.ms += val; return r; }
	/// Return a copy whose elapsed time is decreased by 'val'.
	elapsedMillis operator - (unsigned int val) const  { elapsedMillis r(*this); r.ms += val; return r; }
	/// Return a copy whose elapsed time is decreased by 'val'.
	elapsedMillis operator - (long val) const          { elapsedMillis r(*this); r.ms += val; return r; }
	/// Return a copy whose elapsed time is decreased by 'val'.
	elapsedMillis operator - (unsigned long val) const { elapsedMillis r(*this); r.ms += val; return r; }
	/// Return a copy whose elapsed time is increased by 'val'.
	elapsedMillis operator + (int val) const           { elapsedMillis r(*this); r.ms -= val; return r; }
	/// Return a copy whose elapsed time is increased by 'val'.
	elapsedMillis operator + (unsigned int val) const  { elapsedMillis r(*this); r.ms -= val; return r; }
	/// Return a copy whose elapsed time is increased by 'val'.
	elapsedMillis operator + (long val) const          { elapsedMillis r(*this); r.ms -= val; return r; }
	/// Return a copy whose elapsed time is increased by 'val'.
	elapsedMillis operator + (unsigned long val) const { elapsedMillis r(*this); r.ms -= val; return r; }
};

// elapsedMicros acts as an integer which automatically increments 1 million times
// per second. Useful for creating delays, timeouts, or measuring how long an
// operation takes. You can create as many elapsedMicros variables as needed.
// All of them are independent. Any may be written, modified or read at any time.
/**
 * @brief Tracks elapsed time in microseconds.
 *
 * Works like elapsedMillis but with microsecond resolution. Reading the value (via implicit
 * conversion to unsigned long) yields the number of microseconds that have elapsed since the
 * reference point.
 */
class elapsedMicros
{
private:
	/// Snapshot of system microseconds at the reference point.
	unsigned long long us;
	/// Returns the system microsecond tick count.
	unsigned long long micros() const;
public:
	/// Construct and start at 0 µs elapsed.
	elapsedMicros(void) { us = micros(); }
	/// Construct with an initial elapsed value in microseconds.
	elapsedMicros(unsigned long val) { us = micros() - val; }
	/// Copy-construct from another elapsedMicros.
	elapsedMicros(const elapsedMicros &orig) { us = orig.us; }
	/// Implicitly convert to elapsed microseconds since the reference point.
	operator unsigned long () const { return micros() - us; }
	/// Assign from another instance (copies the reference point).
	elapsedMicros & operator = (const elapsedMicros &rhs) { us = rhs.us; return *this; }
	/// Set current elapsed value to 'val' microseconds.
	elapsedMicros & operator = (unsigned long val) { us = micros() - val; return *this; }
	/// Decrease the reported elapsed time by 'val' microseconds.
	elapsedMicros & operator -= (unsigned long val)      { us += val ; return *this; }
	/// Increase the reported elapsed time by 'val' microseconds.
	elapsedMicros & operator += (unsigned long val)      { us -= val ; return *this; }
	/// Return a copy whose elapsed time is decreased by 'val'.
	elapsedMicros operator - (int val) const           { elapsedMicros r(*this); r.us += val; return r; }
	/// Return a copy whose elapsed time is decreased by 'val'.
	elapsedMicros operator - (unsigned int val) const  { elapsedMicros r(*this); r.us += val; return r; }
	/// Return a copy whose elapsed time is decreased by 'val'.
	elapsedMicros operator - (long val) const          { elapsedMicros r(*this); r.us += val; return r; }
	/// Return a copy whose elapsed time is decreased by 'val'.
	elapsedMicros operator - (unsigned long val) const { elapsedMicros r(*this); r.us += val; return r; }
	/// Return a copy whose elapsed time is increased by 'val'.
	elapsedMicros operator + (int val) const           { elapsedMicros r(*this); r.us -= val; return r; }
	/// Return a copy whose elapsed time is increased by 'val'.
	elapsedMicros operator + (unsigned int val) const  { elapsedMicros r(*this); r.us -= val; return r; }
	/// Return a copy whose elapsed time is increased by 'val'.
	elapsedMicros operator + (long val) const          { elapsedMicros r(*this); r.us -= val; return r; }
	/// Return a copy whose elapsed time is increased by 'val'.
	elapsedMicros operator + (unsigned long val) const { elapsedMicros r(*this); r.us -= val; return r; }
};

#endif // __cplusplus
#endif // elapsedMillis_h