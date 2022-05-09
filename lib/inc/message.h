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

#ifndef MESSAGE_H
#define MESSAGE_H

#include <string>
#include <vector>
#include <Eigen/Dense>

/**
 * @brief This class is a message sent from
 * one agent to another.
 */
class Message
{

public:

    enum Subject
    {
        Information,
        Disable,
        Enable,
        Hit
    };

    Message(unsigned int senderId, unsigned int receiverId, Subject subject, const std::string& textParam = "",
            const std::vector<double>& vecDoubleParam = std::vector<double>(),
            const std::vector<int>& vecIntParam = std::vector<int>());

    /**
     * Destructor
     */
    virtual ~Message();

    /**
     * @return Sender agent's ID.
     */
    unsigned int senderId() const;

    /**
     * @return Receiver agent's ID.
     */
    unsigned int receiverId() const;

    /**
     * @return Get text parameter.
     */
    std::string textParam() const;

    /**
     * @return Get double vector parameter.
     */
    std::vector<double> doubleVecParam() const;

    /**
     * @return Get int vector parameter.
     */
    std::vector<int> intVecParam() const;

    /**
     * @return Subject of the message.
     */
    Subject subject() const;

    /**
     * To string.
     */
    std::string toString() const;

protected:

    unsigned int m_senderId;
    unsigned int m_receiverId;
    Subject m_subject;
    std::string m_textParam;
    std::vector<double> m_doubleVecParam;
    std::vector<int> m_intVecParam;

};

#endif // MESSAGE_H
