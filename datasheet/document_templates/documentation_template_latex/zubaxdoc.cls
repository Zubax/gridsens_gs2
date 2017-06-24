%
% Copyright (c) 2017  Zubax Robotics OU  <info@zubax.com>
%
% Distributed under GPL v3.
%
% Author: Pavel Kirienko <pavel.kirienko@zubax.com>
%
% READ THIS FIRST:
%   In order for this package to work, its local directory must be included into the graphics
%   path list as follows:
%
%       \graphicspath{{document_templates/documentation_template_latex/}}
%
%   Notice the slash at the end, it is required.
%

\NeedsTeXFormat{LaTeX2e}

\ProvidesClass{zubaxdoc}[2017/05/06 Zubax Documentation document class]

\LoadClass[a4paper,onecolumn,openany]{book}

\author{Zubax Robotics O\"U}

\RequirePackage{lipsum}              % Lorem ipsum text generation, useful for testing.
\RequirePackage[pagestyles]{titlesec}
\RequirePackage{titletoc}
\RequirePackage{xcolor}
\RequirePackage{gensymb}
\RequirePackage{graphicx}
\RequirePackage{hyperref}
\RequirePackage{enumitem}
\RequirePackage{multicol}
\RequirePackage{etoolbox}
\RequirePackage{amstext}
\RequirePackage[detect-all]{siunitx}
\RequirePackage[font={bf}]{caption}
\RequirePackage{threeparttablex}
\RequirePackage{tabu}
\RequirePackage{longtable}
\RequirePackage{ifthen}
\RequirePackage{calc}
\RequirePackage{titling}
\RequirePackage[yyyymmdd]{datetime}
\RequirePackage{lastpage}
\RequirePackage{minted}

%
% Minor definitions.
%
\definecolor{colorhyperlink}{RGB}{100,0,0}
\definecolor{colorzubaxred}{RGB}{224,0,0}    % Zubax red logo color

\renewcommand{\dateseparator}{.}

\sisetup{range-phrase = \text{--}}

\newcommand\ZubaxCopyright{\copyright{} \the\year{} \href{http://zubax.com}{\@author}}

% Deferring configuration of the PDF metadata until the point where the document title (and author) are known.
\preto\frontmatter{\hypersetup{
    pdftitle={\@title{} -- \@author},
    pdfauthor={\@author}
}}

% Source code listings highlighting
\usemintedstyle{vs}
\setminted{
frame=leftline,
linenos,
numberblanklines=false,
fontsize=\footnotesize,
xleftmargin=\dimexpr\leftskip-2mm\relax,
framesep=2mm,
autogobble
}
\renewcommand{\theFancyVerbLine}{\scriptsize \texttt \oldstylenums{\arabic{FancyVerbLine}}}

% List styling
\renewcommand{\labelitemi}{\textbullet}
\renewcommand{\labelitemii}{\textbullet}
\renewcommand{\labelitemiii}{\textbullet}
\renewcommand{\labelitemiv}{\textbullet}

\setlist[itemize,2]{leftmargin=2em}
\setlist[itemize,3]{leftmargin=3em}
\setlist[itemize,4]{leftmargin=4em}

\setlist[description]{leftmargin=\leftskip,labelindent=\dimexpr\leftskip+\parindent\relax}
\setlist[enumerate]{leftmargin=\leftskip,itemindent=1.4em,nosep}
\setlist[itemize]{leftmargin=\leftskip,itemindent=1.4em,nosep}

%
% Font specification.
%
\RequirePackage[sfdefault]{roboto}   % Option 'sfdefault' only if the base font of the document is to be sans serif
\RequirePackage[T1]{fontenc}
\RequirePackage{inconsolata}

%
% Page layout (tricky).
%
\newcommand{\DefaultParskip}{0.9em}
\raggedbottom
\RequirePackage[a4paper]{geometry}
\special{papersize=210mm,297mm}  % https://en.wikibooks.org/wiki/LaTeX/Page_Layout
\setlength\columnsep{6mm}        % Multi-column envoronments only
\setlength\headheight{17mm}
\setlength{\parskip}{\DefaultParskip}
\setlength{\abovecaptionskip}{0.2em}
\parindent=0em
\renewcommand{\baselinestretch}{1.2}
\renewcommand{\arraystretch}{1.2}
\leftskip=17mm
\headsep=8mm
\footskip=12mm
\geometry{margin=20mm,top=20mm,bottom=20mm}

% Removes the left margin for the next paragraph
\newcommand{\NoLeftSkip}{\parindent=-\leftskip}

%
% Section formatting
%
\setcounter{secnumdepth}{3}      % Increase this to add deeper structures

\titleformat{\part}[display]{}{\centering\Large Part \thepart}{1em}{
	\centering\huge\bfseries
	}

\titleformat{\chapter}{\huge\bfseries}{}{1em}{\makebox[17mm][l]{\thechapter}\hangindent=17mm}
\titleformat{\section}{\Large\bfseries}{}{1em}{\makebox[17mm][l]{\thesection}\hangindent=17mm}
\titleformat{\subsection}{\bfseries}{}{1em}{\makebox[17mm][l]{\thesubsection}\hangindent=17mm}
\titleformat{\subsubsection}{\itshape}{\thesubsubsection}{1em}{%
	\makebox[17mm][l]{\thesubsubsection}\hangindent=17mm
	}

% Starred sections
\titleformat{name=\chapter,numberless}{\huge\bfseries}{}{1em}{}
\titleformat{name=\section,numberless}{\Large\bfseries}{}{1em}{}

% {left}{before}{after}[right]
\titlespacing{\chapter}{0em}{0em}{0em}
\titlespacing{\section}{0em}{0em}{0em}
\titlespacing{\subsection}{0em}{0em}{0em}
\titlespacing{\subsubsection}{0em}{0em}{0em}

%
% Hyperlinks.
%
\hypersetup{
    colorlinks,
    citecolor=colorhyperlink,
    filecolor=colorhyperlink,
    linkcolor=colorhyperlink,
    urlcolor=colorhyperlink
}

% We don't need to clear double page, ever. Shall the need arise, we'll have to implement a cleaner fix.
\renewcommand{\cleardoublepage}{\clearpage}

%
% Table of contents.
% Note that the title margins are defined in absolute units because we're using different font sizes.
%
\renewcommand{\contentsname}{Table of contents}
\renewcommand{\listfigurename}{List of figures}
\renewcommand{\listtablename}{List of tables}
\setcounter{tocdepth}{2}
\titlecontents{part}[0em]{\centering\bfseries}{}{}{}
\dottedcontents{chapter}[10mm]{\bfseries}{10mm}{1pc}
\dottedcontents{section}[20mm]{}{10mm}{1pc}
\dottedcontents{subsection}[30mm]{\small}{10mm}{1pc}

\dottedcontents{table}[2em]{}{2em}{1pc}
\dottedcontents{figure}[2em]{}{2em}{1pc}

% Custom styling of TOC, LOF, LOT.
\renewcommand\tableofcontents{%
    \thispagestyle{plain}
    \section*{\contentsname}%
    \@starttoc{toc}%
    }
\renewcommand\listoffigures{%
    \thispagestyle{plain}
    \section*{\listfigurename}%
    \@starttoc{lof}%
    }
\renewcommand\listoftables{%
    \thispagestyle{plain}
    \section*{\listtablename}%
    \@starttoc{lot}%
    }

% Make links black in the TOC, LOF, LOT.
\preto\tableofcontents{\hypersetup{linkcolor=black}         \setlength{\parskip}{0em}}
\appto\tableofcontents{\hypersetup{linkcolor=colorhyperlink}\setlength{\parskip}{\DefaultParskip}}

\preto\listoffigures{\hypersetup{linkcolor=black}         \setlength{\parskip}{0em}}
\appto\listoffigures{\hypersetup{linkcolor=colorhyperlink}\setlength{\parskip}{\DefaultParskip}}

\preto\listoftables{\hypersetup{linkcolor=black}         \setlength{\parskip}{0em}}
\appto\listoftables{\hypersetup{linkcolor=colorhyperlink}\setlength{\parskip}{\DefaultParskip}}

%
% Table macros.
%
% Use this wrapper environment to define tables in it.
% This environment provides proper placement of the table within the page, and the properly positioned caption.
\newenvironment{ZubaxTableWrapper}[1]{
    \NoLeftSkip
    \begin{minipage}{\textwidth}  % Minipage is needed to prevent page breaks after the caption
        \begin{ThreePartTable}
            % https://tex.stackexchange.com/questions/3243/why-should-a-table-caption-be-placed-above-the-table
            \captionof{table}{#1}
}{
        \end{ThreePartTable}
    \end{minipage}
}

\newenvironment{ZubaxCompactTable}[1]{
    \begin{tabu}{#1}
        \hline
        \everyrow{\hline}
        \rowfont{\bfseries}
}{
    \end{tabu}
}

% Use this environment to define a table within the wrapper environment defined above.
\newenvironment{ZubaxWrappedTable}[1]{
    \NoLeftSkip
    \begin{tabu} to \textwidth {#1}
        \hline
        \everyrow{\hline}
        \rowfont{\bfseries}
}{
    \end{tabu}
}

% This is a shortcut that places a table within the wrapper environment.
% Use it in all cases unless you need to define table notes, or if you need to customize something.
\newenvironment{ZubaxSimpleTable}[2]{
    \begin{ZubaxTableWrapper}{#1}
        \begin{ZubaxWrappedTable}{#2}
}{
        \end{ZubaxWrappedTable}
    \end{ZubaxTableWrapper}
}

%
% Page styling.
%
\newcommand\ChapterTitleWithNumber{%
    \ifthenelse{\equal{\thechapter}{0}}%
    {}%
    {\thechapter.\ }%
    \ifdefempty{\chaptertitle}%
    {\sectiontitle}%
    {\chaptertitle}%
    }

\newcommand\PageNumberWithTotal{\thepage\if@mainmatter/\pageref*{LastPage}\fi}

\newcommand\ZubaxLogoBlack[1][15mm]{\includegraphics[width=#1]{zubax_logo_black}}
\newcommand\ZubaxLogo[1][15mm]{\includegraphics[width=#1]{zubax_logo}}

\newcommand\VerboseLinkToSupportForum{Support \& feedback: \href{https://productforums.zubax.com}{productforums.zubax.com}}

\newcommand\TitleWithDateBefore{\begin{footnotesize}\today\quad\end{footnotesize}{\@title}}
\newcommand\TitleWithDateAfter{{\@title}\begin{footnotesize}\quad\today\end{footnotesize}}

\renewpagestyle{plain}{
	\sethead%
	    [\TitleWithDateAfter][][\ZubaxLogoBlack]     % even
	    {\ZubaxLogoBlack}{}{\TitleWithDateBefore}    % odd

	\setfoot%
	    [\PageNumberWithTotal][\small\VerboseLinkToSupportForum][\small\ZubaxCopyright]
	    {\small\ZubaxCopyright}{\small\VerboseLinkToSupportForum}{\PageNumberWithTotal}

	\headrule
	\footrule
    }
\newpagestyle{main}{
	\sethead%
	    [\TitleWithDateAfter][][\ZubaxLogoBlack]     % even
	    {\ZubaxLogoBlack}{}{\TitleWithDateBefore}    % odd

	\setfoot%
	    [\PageNumberWithTotal][][\small\ChapterTitleWithNumber]
	    {\small\ChapterTitleWithNumber}{}{\PageNumberWithTotal}

	\headrule
	\footrule
    }
\newpagestyle{titlepage}{
	\setfoot{\ZubaxCopyright}{}{\VerboseLinkToSupportForum}
	\footrule
    }

% Use the same style on pages that open new chapters.
\patchcmd{\chapter}{plain}{main}{}{}

% The document will switch to the proper page style automatically.
\preto\frontmatter{\twocolumn\pagestyle{plain}}
\preto\mainmatter{\onecolumn\pagestyle{main}}
\preto\backmatter{\onecolumn\pagestyle{plain}}

%
% Title page environment.
% Use "\BeginRightColumn" to switch to the right column.
%
\newcommand{\BeginRightColumn}{\vfill\eject}

\newcommand{\RenderTitlePageHeader}{%
    \renewcommand{\baselinestretch}{1}
    \begin{minipage}[b][40mm]{0.65\textwidth}
        \vfill
        \begin{flushleft}\begin{Huge}\@title\end{Huge}\end{flushleft}
        \vfill
        Revision \today
    \end{minipage}
    \hfill
    \begin{minipage}[b]{0.3\textwidth}
        \begin{flushright}
        \begin{small}
            \ZubaxLogo[50mm]\\\vspace{3mm}
            \textbf{\@author}\\
            Narva mnt. 5, Tallinn 10117, Estonia\\
            \href{mailto:info@zubax.com}{info@zubax.com}\\
            Q\&{}A: \href{https://productforums.zubax.com}{productforums.zubax.com}
        \end{small}
        \end{flushright}
    \end{minipage}
    % Without \hfill here, TeX complains that hbox is underfull,
    % despite the fact that the rule spans for \textwidth here. I have no idea why.
    \rule{\textwidth}{.4pt}\hfill\vspace{3mm}
    }

\renewenvironment{titlepage}{%
    \leftskip=0mm\parindent=0mm
    \pagestyle{titlepage}\thispagestyle{titlepage}
    \twocolumn[%
        \begin{@twocolumnfalse}
            \RenderTitlePageHeader
        \end{@twocolumnfalse}
        ]
    }{%
    \clearpage
    }

\endinput
