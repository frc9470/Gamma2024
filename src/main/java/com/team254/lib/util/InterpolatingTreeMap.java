package com.team254.lib.util;

import java.util.Map;
import java.util.TreeMap;

/**
 * Interpolating Tree Maps are used to get values at points that are not defined by making a guess from points that are
 * defined. This uses linear interpolation.
 *
 * @param <K> The type of the key (must implement InverseInterpolable)
 * @param <V> The type of the value (must implement Interpolable)
 */
public class InterpolatingTreeMap<K extends InverseInterpolable<K> & Comparable<K>, V extends Interpolable<V>>
        extends TreeMap<K, V> {
    private static final long serialVersionUID = 8347275262778054124L;

    final int max_;

    public InterpolatingTreeMap(int maximumSize) {
        max_ = maximumSize;
    }

    public InterpolatingTreeMap() {
        this(0);
    }

    public InterpolatingTreeMap(K[] keys, V[] values) {
        this(0);
        for (int i = 0; i < keys.length; i++) {
            put(keys[i], values[i]);
        }
    }

    public InterpolatingTreeMap(K[][] values) {
        this(0);
        // if the types KV dont match, throw an error

        if (values[0].length != 2) {
            throw new IllegalArgumentException("Input array does not have 2 columns");
        }
        for (K[] value : values) {
            put(value[0], (V) value[1]);
        }
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ofDouble(double[][] values) {
        if (values[0].length != 2) {
            throw new IllegalArgumentException("Input array does not have 2 columns");
        }
        InterpolatingDouble[] keys = new InterpolatingDouble[values.length];
        InterpolatingDouble[] vals = new InterpolatingDouble[values.length];
        for (int i = 0; i < values.length; i++) {
            keys[i] = new InterpolatingDouble(values[i][0]);
            vals[i] = new InterpolatingDouble(values[i][1]);
        }
        return new InterpolatingTreeMap<>(keys, vals);
    }

    /**
     * Inserts a key value pair, and trims the tree if a max size is specified
     *
     * @param key   Key for inserted data
     * @param value Value for inserted data
     * @return the value
     */
    @Override
    public V put(K key, V value) {
        if (max_ > 0 && max_ <= size()) {
            // "Prune" the tree if it is oversize
            K first = firstKey();
            remove(first);
        }

        super.put(key, value);

        return value;
    }

    @Override
    public void putAll(Map<? extends K, ? extends V> map) {
        System.out.println("Unimplemented Method");
    }

    /**
     * @param key Lookup for a value (does not have to exist)
     * @return V or null; V if it is Interpolable or exists, null if it is at a bound and cannot average
     */
    public V getInterpolated(K key) {
        V gotval = get(key);
        if (gotval == null) {
            // get surrounding keys for interpolation
            K topBound = ceilingKey(key);
            K bottomBound = floorKey(key);

            // if attempting interpolation at ends of tree, return the nearest data point
            if (topBound == null && bottomBound == null) {
                return null;
            } else if (topBound == null) {
                return get(bottomBound);
            } else if (bottomBound == null) {
                return get(topBound);
            }

            // get surrounding values for interpolation
            V topElem = get(topBound);
            V bottomElem = get(bottomBound);
            return bottomElem.interpolate(topElem, bottomBound.inverseInterpolate(topBound, key));
        } else {
            return gotval;
        }
    }
}