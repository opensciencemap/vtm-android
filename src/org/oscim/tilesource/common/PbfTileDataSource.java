/*
 * Copyright 2012 Hannes Janetzek
 *
 * This program is free software: you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */
package org.oscim.tilesource.common;

import java.io.IOException;
import java.io.InputStream;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;

import org.oscim.layers.tile.MapTile;
import org.oscim.tilesource.ITileCache;
import org.oscim.tilesource.ITileDataSink;
import org.oscim.tilesource.ITileDataSource;
import org.oscim.tilesource.TileSource;

import android.util.Log;

/**
 *
 *
 */
public abstract class PbfTileDataSource implements ITileDataSource {
	private static final String TAG = PbfTileDataSource.class.getName();

	protected LwHttp mConn;
	protected final PbfDecoder mTileDecoder;
	protected final ITileCache mTileCache;

	public PbfTileDataSource(TileSource tileSource, PbfDecoder tileDecoder) {
		mTileDecoder = tileDecoder;
		mTileCache = tileSource.tileCache;
	}


	@Override
	public QueryResult executeQuery(MapTile tile, ITileDataSink sink) {
		boolean success = true;

		ITileCache.TileWriter cacheWriter = null;

		if (mTileCache != null) {
			ITileCache.TileReader c = mTileCache.getTile(tile);
			if (c == null) {
				// create new cache entry
				cacheWriter = mTileCache.writeTile(tile);
				mConn.setOutputStream(cacheWriter.getOutputStream());
			}
			else {
				try {
					if (mTileDecoder.decode(tile, sink, c.getInputStream(), c.getBytes()))
						return QueryResult.SUCCESS;

				} catch (IOException e) {
					e.printStackTrace();
				}
				Log.d(TAG, tile + " cache read failed");
			}
		}

		try {
			InputStream is;
			if (!mConn.sendRequest(tile)) {
				Log.d(TAG, tile + " Request Failed");
				success = false;
			} else if ((is = mConn.readHeader()) != null) {
				boolean ok = mTileDecoder.decode(tile, sink, is, mConn.getContentLength());
				if (!ok)
					Log.d(TAG, tile + " failed");
			} else {
				Log.d(TAG, tile + " Network Error");
				success = false;
			}
		} catch (SocketException e) {
			Log.d(TAG, tile + " Socket exception: " + e.getMessage());
			success = false;
		} catch (SocketTimeoutException e) {
			Log.d(TAG, tile + " Socket Timeout");
			success = false;
		} catch (UnknownHostException e) {
			Log.d(TAG, tile + " No Network");
			success = false;
		} catch (Exception e) {
			e.printStackTrace();
			success = false;
		}

		mConn.requestCompleted();

		if (cacheWriter != null)
			cacheWriter.complete(success);

		if (!success)
			mConn.close();

		return success ? QueryResult.SUCCESS : QueryResult.FAILED;
	}

	@Override
	public void destroy() {
		mConn.close();
	}
}
