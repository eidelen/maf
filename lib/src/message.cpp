/****************************************************************************
** Copyright (c) 2021 Adrian Schneider
**
** Permission is hereby granted, free of charge, to any person obtaining a
** copy of this software and associated documentation files (the "Software"),
** to deal in the Software without restriction, including without limitation
** the rights to use, copy, modify, merge, publish, distribute, sublicense,
** and/or sell copies of the Software, and to permit persons to whom the
** Software is furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in
** all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
** DEALINGS IN THE SOFTWARE.
**
*****************************************************************************/

#include "message.h"

Message::Message(unsigned int senderId, unsigned int receiverId, Message::Subject subject, const std::string &textParam,
                 const std::vector<double>& vecDoubleParam, const std::vector<int> &vecIntParam)
    : m_senderId(senderId), m_receiverId(receiverId), m_subject(subject), m_textParam(textParam),
      m_doubleVecParam(vecDoubleParam), m_intVecParam(vecIntParam)
{

}

Message::~Message()
{

}

unsigned int Message::senderId() const
{
    return m_senderId;
}

unsigned int Message::receiverId() const
{
    return m_receiverId;
}

std::string Message::textParam() const
{
    return m_textParam;
}

std::vector<double> Message::doubleVecParam() const
{
    return m_doubleVecParam;
}

std::vector<int> Message::intVecParam() const
{
    return m_intVecParam;
}

Message::Subject Message::subject() const
{
    return m_subject;
}

std::string Message::toString() const
{
    std::ostringstream s;
    s << "Sender: " << this->m_senderId << ", Receiver: " << this->m_receiverId;

    switch(this->subject())
    {
    case Message::Information:
        s << ", Subject: Information";
        break;
    case Message::Disable:
        s << ", Subject: Disable";
        break;
    case Message::Enable:
        s << ", Subject: Enable";
        break;
    case Message::Hit:
        s << ", Subject: Hit";
        break;
    }

    return s.str();
}

