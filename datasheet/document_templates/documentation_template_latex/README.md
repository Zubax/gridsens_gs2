Zubax documentation template
============================

Use this LaTeX template to write documentation.

Consider the following usage example below; also refer to the existing documents.

```tex
\documentclass{zubaxdoc}                                           % You can use a symlink to refer to the class
\graphicspath{{document_templates/documentation_template_latex/}}  % Path to the class directory must be provided
\title{A demo document}

\begin{document}

\frontmatter

\begin{titlepage}
\section*{Overview}
Here goes a brief overview of the document.
\BeginRightColumn   % Use this to switch to the right column early.
\section*{Capabilities}
Some content on the right column.
\end{titlepage}

\tableofcontents    % Optional
\listoffigures      % Optional
\listoftables       % Optional

\mainmatter

\part{User manual}        % Parts are optional, they do not affect the structure

\chapter{Introduction}

\lipsum[1-2]

\section{First section of the introduction}

\lipsum[3-4]

\subsection{This is a subsection}

\begin{minted}{python}
class Monitor(object):
    def __init__(self, event):
        self.message = event.message
        self.transfer = event.transfer
        self.node = event.node

    def on_message(self):
        pass
\end{minted}

\begin{minted}{c++}
    constexpr inline Scalar convertKelvinToCelsius(Const kelvin)
    {
        return kelvin - 273.15F;
    }

    constexpr inline Scalar convertCelsiusToKelvin(Const kelvin)
    {
        return kelvin + 273.15F;
    }
\end{minted}

\subsubsection{This is a sub-subsection}

\lipsum[5-6]

\end{document}
```
