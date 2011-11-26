/*
 * Copyright (C) 2007 Alp Toker <alp@atoker.com>
 * Copyright (C) 2007 Apple Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public License
 * along with this library; see the file COPYING.LIB.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#include "config.h"
#include "PrintContext.h"

#include "GraphicsContext.h"
#include "Frame.h"
#include "FrameView.h"
#include "RenderView.h"

using namespace WebCore;

namespace WebCore {

PrintContext::PrintContext(Frame* frame)
    : m_frame(frame)
{
}

PrintContext::~PrintContext()
{
    m_pageRects.clear();
}

int PrintContext::pageCount() const
{
    return m_pageRects.size();
}

const IntRect& PrintContext::pageRect(int pageNumber) const
{
    return m_pageRects[pageNumber];
}

void PrintContext::computePageRects(const FloatRect& printRect, float headerHeight, float footerHeight, float userScaleFactor, float& outPageHeight)
{
    m_pageRects.clear();
    outPageHeight = 0;

    if (!m_frame->document() || !m_frame->view() || !m_frame->document()->renderer())
        return;

    RenderView* root = toRenderView(m_frame->document()->renderer());

    if (!root) {
        LOG_ERROR("document to be printed has no renderer");
        return;
    }

    float ratio = printRect.height() / printRect.width();

    float pageWidth  = (float)root->rightLayoutOverflow();
    float pageHeight = pageWidth * ratio;
    outPageHeight = pageHeight;   // this is the height of the page adjusted by margins
    pageHeight -= headerHeight + footerHeight;

    if (pageHeight <= 0) {
        LOG_ERROR("pageHeight has bad value %.2f", pageHeight);
        return;
    }

    computePageRectsWithPageSize(FloatSize(pageWidth, pageHeight), userScaleFactor);
}

void PrintContext::computePageRectsWithPageSize(const FloatSize& pageSizeInPixels, float userScaleFactor)
{
    RenderView* root = toRenderView(m_frame->document()->renderer());

    if (!root) {
        LOG_ERROR("document to be printed has no renderer");
        return;
    }

    if (userScaleFactor <= 0) {
        LOG_ERROR("userScaleFactor has bad value %.2f", userScaleFactor);
        return;
    }

    float currPageHeight = pageSizeInPixels.height() / userScaleFactor;
    float docHeight = root->layer()->height();
    float currPageWidth = pageSizeInPixels.width() / userScaleFactor;
    m_docWidth = currPageWidth;

    // always return at least one page, since empty files should print a blank page
    float printedPagesHeight = 0;
    do {
        float proposedBottom = std::min(docHeight, printedPagesHeight + pageSizeInPixels.height());
        m_frame->view()->adjustPageHeight(&proposedBottom, printedPagesHeight, proposedBottom, printedPagesHeight);
        currPageHeight = max(1.0f, proposedBottom - printedPagesHeight);

        m_pageRects.append(IntRect(0, (int)printedPagesHeight, (int)currPageWidth, (int)currPageHeight));
        printedPagesHeight += currPageHeight;
    } while (printedPagesHeight < docHeight);
}

void PrintContext::begin(float width)
{
    // By imaging to a width a little wider than the available pixels,
    // thin pages will be scaled down a little, matching the way they
    // print in IE and Camino. This lets them use fewer sheets than they
    // would otherwise, which is presumably why other browsers do this.
    // Wide pages will be scaled down more than this.
    const float PrintingMinimumShrinkFactor = 1.25f;

    // This number determines how small we are willing to reduce the page content
    // in order to accommodate the widest line. If the page would have to be
    // reduced smaller to make the widest line fit, we just clip instead (this
    // behavior matches MacIE and Mozilla, at least)
    const float PrintingMaximumShrinkFactor = 2.0f;

    float minLayoutWidth = width * PrintingMinimumShrinkFactor;
    float maxLayoutWidth = width * PrintingMaximumShrinkFactor;

    // FIXME: This will modify the rendering of the on-screen frame.
    // Could lead to flicker during printing.
    m_frame->setPrinting(true, minLayoutWidth, maxLayoutWidth, true);
}

void PrintContext::spoolPage(GraphicsContext& ctx, int pageNumber, float width)
{
    IntRect pageRect = m_pageRects[pageNumber];
    float scale = width / pageRect.width();

    ctx.save();
    ctx.scale(FloatSize(scale, scale));
    ctx.translate(-pageRect.x(), -pageRect.y());
    ctx.clip(pageRect);
    m_frame->view()->paintContents(&ctx, pageRect);
    ctx.restore();
}

void PrintContext::spoolPageBand(GraphicsContext& ctx, int pageNumber, float width, const IntRect& _bandRect, float ext_scale)
{
    if (pageNumber >= (int)m_pageRects.size())
        return;
    IntRect pageRect = m_pageRects[pageNumber];
    float scale = width / pageRect.width();
    float finalScale = scale * ext_scale;
    IntRect bandRect(_bandRect);

    int maxY = pageRect.y() + pageRect.height();
    int maxX = pageRect.x() + pageRect.width();

    pageRect.setY(pageRect.y() + (int) (bandRect.y()  / finalScale));
    pageRect.setHeight((int) (bandRect.height() / finalScale));

    if ((pageRect.y() + pageRect.height()) > maxY)
    {
        pageRect.setHeight(maxY - pageRect.y());
    }

    pageRect.setX(pageRect.x() + (int) (bandRect.x() / finalScale));
    pageRect.setWidth((int) (bandRect.width() / finalScale));

    if ((pageRect.x() + pageRect.width()) > maxX)
    {
        pageRect.setWidth(maxX - pageRect.x());
    }

    fprintf(stderr, "rendering page %d rect: x,y=%d,%d w,h=%d,%d\n", pageNumber, pageRect.x(), pageRect.y(), pageRect.width(), pageRect.height());

    ctx.save();
    ctx.scale(FloatSize(finalScale, finalScale));
    ctx.translate(-pageRect.x(), -pageRect.y());

    pageRect.inflateX(1);
    pageRect.inflateY(1);
    
    ctx.clip(pageRect);
    m_frame->view()->paintContents(&ctx, pageRect);
    ctx.restore();
}

int PrintContext::getPageHeight(int pageNumber, const FloatRect& printRect, float userScaleFactor)
{
    if (pageNumber >= (int)m_pageRects.size())
        return(0);

    float currPageWidth = m_docWidth / userScaleFactor;

    int pageHeight = (int)( m_pageRects[pageNumber].height() * (printRect.width() / currPageWidth));

    return (int)( m_pageRects[pageNumber].height() * (printRect.width() / currPageWidth));
}

void PrintContext::end()
{
    m_frame->setPrinting(false, 0, 0, true);
}

static RenderBoxModelObject* enclosingBoxModelObject(RenderObject* object)
{

    while (object && !object->isBoxModelObject())
        object = object->parent();
    if (!object)
        return 0;
    return toRenderBoxModelObject(object);
}

int PrintContext::pageNumberForElement(Element* element, const FloatSize& pageSizeInPixels)
{
    // Make sure the element is not freed during the layout.
    RefPtr<Element> elementRef(element);
    element->document()->updateLayout();

    RenderBoxModelObject* box = enclosingBoxModelObject(element->renderer());
    if (!box)
        return -1;

    Frame* frame = element->document()->frame();
    FloatRect pageRect(FloatPoint(0, 0), pageSizeInPixels);
    PrintContext printContext(frame);
    printContext.begin(pageRect.width());
    printContext.computePageRectsWithPageSize(pageSizeInPixels, 1);

    int top = box->offsetTop();
    int left = box->offsetLeft();
    for (int pageNumber = 0; pageNumber < printContext.pageCount(); pageNumber++) {
        const IntRect& page = printContext.pageRect(pageNumber);
        if (page.x() <= left && left < page.right() && page.y() <= top && top < page.bottom())
            return pageNumber;
    }
    printContext.end();
    return -1;
}

int PrintContext::numberOfPages(Frame* frame, const FloatSize& pageSizeInPixels)
{
    frame->document()->updateLayout();

    FloatRect pageRect(FloatPoint(0, 0), pageSizeInPixels);
    PrintContext printContext(frame);
    printContext.begin(pageRect.width());
    printContext.computePageRectsWithPageSize(pageSizeInPixels, 1);
    printContext.end();
    return printContext.pageCount();
}

}
