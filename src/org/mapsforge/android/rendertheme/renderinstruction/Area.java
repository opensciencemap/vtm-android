/*
 * Copyright 2010, 2011, 2012 mapsforge.org
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
package org.mapsforge.android.rendertheme.renderinstruction;

import org.mapsforge.android.rendertheme.IRenderCallback;
import org.mapsforge.android.rendertheme.RenderThemeHandler;
import org.mapsforge.core.Tag;
import org.xml.sax.Attributes;

import android.graphics.Color;

/**
 * Represents a closed polygon on the map.
 */
public final class Area extends RenderInstruction {
	/**
	 * @param elementName
	 *            the name of the XML element.
	 * @param attributes
	 *            the attributes of the XML element.
	 * @param level
	 *            the drawing level of this instruction.
	 * @return a new Area with the given rendering attributes.
	 */
	public static Area create(String elementName, Attributes attributes, int level) {
		String src = null;
		int fill = Color.BLACK;
		int stroke = Color.TRANSPARENT;
		float strokeWidth = 0;
		int fade = -1;
		int blend = -1;
		int blendFill = Color.BLACK;
		String style = null;

		for (int i = 0; i < attributes.getLength(); ++i) {
			String name = attributes.getLocalName(i);
			String value = attributes.getValue(i);
			if ("name".equals(name))
				style = value;
			else if ("src".equals(name)) {
				src = value;
			} else if ("fill".equals(name)) {
				fill = Color.parseColor(value);
			} else if ("stroke".equals(name)) {
				stroke = Color.parseColor(value);
			} else if ("stroke-width".equals(name)) {
				strokeWidth = Float.parseFloat(value);
			} else if ("fade".equals(name)) {
				fade = Integer.parseInt(value);
			} else if ("blend".equals(name)) {
				blend = Integer.parseInt(value);
			} else if ("blend-fill".equals(name)) {
				blendFill = Color.parseColor(value);
			} else {
				RenderThemeHandler.logUnknownAttribute(elementName, name, value, i);
			}
		}

		validate(strokeWidth);
		return new Area(style, src, fill, stroke, strokeWidth, fade, level, blend,
				blendFill);
	}

	private static void validate(float strokeWidth) {
		if (strokeWidth < 0) {
			throw new IllegalArgumentException("stroke-width must not be negative: "
					+ strokeWidth);
		}
	}

	private Area(String style, String src, int fill, int stroke, float strokeWidth,
			int fade, int level, int blend, int blendFill) {
		super();
		this.style = style;

		// if (fill == Color.TRANSPARENT) {
		// paintFill = null;
		// } else {
		// paintFill = new Paint(Paint.ANTI_ALIAS_FLAG);
		// if (src != null) {
		// Shader shader = BitmapUtils.createBitmapShader(src);
		// paintFill.setShader(shader);
		// }
		// paintFill.setStyle(Style.FILL);
		// paintFill.setColor(fill);
		// paintFill.setStrokeCap(Cap.ROUND);
		// }
		//
		// if (stroke == Color.TRANSPARENT) {
		// paintOutline = null;
		// } else {
		// paintOutline = new Paint(Paint.ANTI_ALIAS_FLAG);
		// paintOutline.setStyle(Style.STROKE);
		// paintOutline.setColor(stroke);
		// paintOutline.setStrokeCap(Cap.ROUND);
		// }

		// if (stroke == Color.TRANSPARENT) {
		// stroke = null;
		// } else{
		// stroke = new Line()
		// }

		color = new float[4];
		color[0] = (fill >> 16 & 0xff) / 255.0f;
		color[1] = (fill >> 8 & 0xff) / 255.0f;
		color[2] = (fill >> 0 & 0xff) / 255.0f;
		color[3] = (fill >> 24 & 0xff) / 255.0f;

		if (blend > 0) {
			blendColor = new float[4];
			blendColor[0] = (blendFill >> 16 & 0xff) / 255.0f;
			blendColor[1] = (blendFill >> 8 & 0xff) / 255.0f;
			blendColor[2] = (blendFill >> 0 & 0xff) / 255.0f;
			blendColor[3] = (blendFill >> 24 & 0xff) / 255.0f;
		} else {
			blendColor = null;
		}

		this.blend = blend;
		this.strokeWidth = strokeWidth;
		this.fade = fade;
		this.level = level;
	}

	@Override
	public void renderWay(IRenderCallback renderCallback, Tag[] tags) {
		renderCallback.renderArea(this, this.level);
	}

	// @Override
	// public void scaleStrokeWidth(float scaleFactor) {
	// // if (paintOutline != null) {
	// // paintOutline.setStrokeWidth(strokeWidth * scaleFactor);
	// // }
	// }

	public String style;
	/**
	 * 
	 */
	private final int level;
	/**
	 * 
	 */
	// public final Paint paintFill;
	/**
	 * 
	 */
	// public final Paint paintOutline;
	/**
	 * 
	 */
	public final float strokeWidth;
	/**
	 * 
	 */
	public final float color[];
	/**
	 * 
	 */
	public final int fade;

	public final float blendColor[];

	public final int blend;
}