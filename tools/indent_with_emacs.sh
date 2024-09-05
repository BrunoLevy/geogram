#! /bin/sh
":"; exec emacs --no-site-file --script "$0" -- "$@" # -*- mode:emacs-lisp -*-

;; emacs options:
(setq make-backup-files nil)
(setq next-line-add-newlines nil)

(setq-default indent-tabs-mode nil)
(setq-default c-basic-offset 4)
(setq c-default-style "linux")

(add-hook 'c-mode-hook
  (lambda()
     (c-set-offset 'arglist-close 'c-lineup-close-paren)
  )
)

(add-hook 'c++-mode-hook
  (lambda()
     (c-set-offset 'arglist-close 'c-lineup-close-paren)
  )
)


;; open file:
(find-file (nth 1 argv))

;; process file:
(delete-trailing-whitespace)
(untabify (point-min) (point-max))
(indent-region (point-min) (point-max))

;; save file:
(save-buffer)
